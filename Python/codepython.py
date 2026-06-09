#!/usr/bin/env python3
# save_per_node_auto_ndjson.py
#
# AUTO logger for the combined master:
# - USBMUX transport: COBS(frame) + 0x00 delimiter, CRC16/MODBUS inside decoded frame
# - USBMUX type=0x01: FAST raw chunks -> reassemble into frames and write to MIC/VIB/RES BIN
# - USBMUX type=0x02: SLOW snapshots -> per-UID NDJSON + raw audit file + stats + manifest
#
# Each run creates a NEW session folder: out_root/YYYYMMDD_HHMMSS/
#
# Usage:
#   pip install pyserial
#   python save_per_node_auto_ndjson.py --port COM19 --out out_capture

import os
import time
import json
import struct
import argparse
from pathlib import Path
from datetime import datetime

import serial


# ---------------- USBMUX helpers ----------------
# decoded layout:
#  [type:u8][seq:u16le][len:u16le][t_us:u32le][data:len][crc:u16le]
# crc = CRC16/MODBUS over everything before crc (type..data)

def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else (crc >> 1)
    return crc & 0xFFFF


def cobs_decode(inp: bytes) -> bytes:
    out = bytearray()
    i = 0
    n = len(inp)
    while i < n:
        code = inp[i]
        if code == 0:
            raise ValueError("COBS code=0")
        i += 1
        copy_len = code - 1
        if i + copy_len > n and code != 1:
            raise ValueError("COBS overrun")
        out += inp[i:i + copy_len]
        i += copy_len
        if code != 0xFF and i < n:
            out.append(0)
    return bytes(out)


# ---------------- FAST raw frame parsing ----------------
PRE = b"\x55" * 16
SOF_MIC = 0xA5
SOF_VIB = 0xA6
SOF_RES = 0xA7

MIC_PAYLOAD = 960
VIB_PAYLOAD = 192

MIC_LEN = 16 + 1 + 1 + MIC_PAYLOAD + 2
VIB_LEN = 16 + 1 + 1 + VIB_PAYLOAD + 2
RES_LEN = 16 + 1 + 1 + 2 + 2 + 2  # PRE + SOF + epoch + apply_u16 + cycle_u16 + crc16

ALLOWED_SOF = {SOF_MIC, SOF_VIB, SOF_RES}


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8) & 0xFFFF
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc & 0xFFFF


class FastFrameAssembler:
    def __init__(self):
        self.buf = bytearray()

    def feed(self, chunk: bytes):
        self.buf.extend(chunk)

    def pop_frames(self):
        frames = []
        b = self.buf

        while True:
            p = b.find(PRE)
            if p < 0:
                if len(b) > 15:
                    del b[:-15]
                break

            if p > 0:
                del b[:p]

            if len(b) < 17:
                break

            sof = b[16]
            if sof not in ALLOWED_SOF:
                del b[0:1]
                continue

            if sof == SOF_MIC:
                need = MIC_LEN
            elif sof == SOF_VIB:
                need = VIB_LEN
            else:
                need = RES_LEN

            if len(b) < need:
                break

            frame = bytes(b[:need])

            crc_rx = frame[-2] | (frame[-1] << 8)
            crc_calc = crc16_ccitt_false(frame[16:-2])
            if crc_rx != crc_calc:
                del b[0:1]
                continue

            frames.append(frame)
            del b[:need]

        return frames


# ---------------- SLOW snapshot parsing ----------------
# payload (USBMUX type=0x02 data):
# [schema_ver:u8][msg_type:u8]
# [seq:u32le][ts_ms:u32le]
# [uid:6][addr:u8][status:u16le][count:u8]
# repeat count:
#   [ch:u8][value:i32le][scale:u16le][unit:u8]

USBMSG_SLOW_SNAPSHOT = 0x01


def uid_hex(uid_bytes: bytes) -> str:
    # match your master uid_to_str order: uid[5]..uid[0]
    return "".join(f"{uid_bytes[i]:02X}" for i in (5, 4, 3, 2, 1, 0))


def make_session_dir(root: str) -> Path:
    rootp = Path(root)
    rootp.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    sess = rootp / stamp
    sess.mkdir(parents=True, exist_ok=False)
    return sess


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True, help="COM19 / /dev/ttyACM0 ...")
    ap.add_argument("--baud", type=int, default=2000000, help="USB CDC baud (ignored by CDC but kept)")
    ap.add_argument("--out", default="out_capture", help="root output folder (session subfolder is created each run)")
    ap.add_argument("--flush-ms", type=int, default=1000, help="stats/manifest flush interval")
    args = ap.parse_args()

    outdir = make_session_dir(args.out)

    # --- slow outputs (ndjson + raw + stats + manifest)
    slow_dir = outdir / "slow"
    slow_dir.mkdir(exist_ok=True)

    raw_slow_path = slow_dir / "raw_slow.bin"          # raw audit: only USBMUX cobs frames for type=0x02, with 0x00 delimiter
    stats_path = slow_dir / "stats_slow.json"
    manifest_path = slow_dir / "manifest.json"

    stats = {
        "frames_total": 0,
        "frames_ok": 0,
        "crc_fail": 0,
        "cobs_fail": 0,
        "short_fail": 0,
        "unknown_msg": 0,
        "seq_gaps": 0,
        "per_uid": {}  # uid -> {"ok":..,"seq_gaps":..}
    }
    last_seq_by_uid = {}
    seen_uids = set()

    def per_uid_stats(uid: str):
        if uid not in stats["per_uid"]:
            stats["per_uid"][uid] = {"ok": 0, "seq_gaps": 0}
        return stats["per_uid"][uid]

    manifest = {
        "session_out": str(outdir),
        "bus": "slow",
        "transport": "USBMUX(COBS(frame)+0x00), frame contains CRC16_MODBUS",
        "schema_ver": 1,
        "msg_types": {"1": "SLOW_SNAPSHOT"},
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "nodes": []
    }
    manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")

    # --- fast outputs
    fast_dir = outdir / "fast_raw"
    fast_dir.mkdir(exist_ok=True)
    f_mic = open(fast_dir / "MIC_fast.bin", "wb", buffering=0)
    f_vib = open(fast_dir / "VIB_fast.bin", "wb", buffering=0)
    f_res = open(fast_dir / "RESYNC_fast.bin", "wb", buffering=0)

    assembler = FastFrameAssembler()

    # --- IO
    ser = serial.Serial(args.port, args.baud, timeout=0.05)
    ser.reset_input_buffer()
    rx_buf = bytearray()

    last_flush = time.time()
    good_umx = bad_umx = 0
    good_fast_frames = 0

    print(f"[RUN] port={args.port} session_out={outdir.resolve()}")
    print("[INFO] SLOW: per-UID NDJSON + raw_slow.bin + stats/manifest. FAST: MIC/VIB/RES BIN. Ctrl+C to stop.")

    try:
        with open(raw_slow_path, "wb", buffering=0) as raw_slow_f:
            while True:
                data = ser.read(8192)
                if data:
                    rx_buf.extend(data)

                while True:
                    z = rx_buf.find(b"\x00")
                    if z < 0:
                        if len(rx_buf) > 200000:
                            del rx_buf[:-50000]
                        break

                    frame_cobs = bytes(rx_buf[:z])
                    del rx_buf[:z + 1]

                    if not frame_cobs:
                        continue

                    # decode USBMUX
                    try:
                        dec = cobs_decode(frame_cobs)
                    except Exception:
                        bad_umx += 1
                        continue

                    if len(dec) < (1 + 2 + 2 + 4 + 2):
                        bad_umx += 1
                        continue

                    typ = dec[0]
                    ln = dec[3] | (dec[4] << 8)

                    if len(dec) != 1 + 2 + 2 + 4 + ln + 2:
                        bad_umx += 1
                        continue

                    crc_rx = dec[-2] | (dec[-1] << 8)
                    if crc_rx != crc16_modbus(dec[:-2]):
                        bad_umx += 1
                        continue

                    payload = dec[9:9 + ln]
                    good_umx += 1

                    # FAST
                    if typ == 0x01:
                        assembler.feed(payload)
                        frames = assembler.pop_frames()
                        for fr in frames:
                            sof = fr[16]
                            if sof == SOF_MIC:
                                f_mic.write(fr)
                            elif sof == SOF_VIB:
                                f_vib.write(fr)
                            else:
                                f_res.write(fr)
                            good_fast_frames += 1
                        continue

                    # SLOW
                    if typ != 0x02:
                        continue

                    # raw audit for slow: store original cobs frame + delimiter (exactly like your slow logger does for cobs frames)
                    stats["frames_total"] += 1
                    raw_slow_f.write(frame_cobs + b"\x00")

                    # parse slow snapshot
                    if len(payload) < 20:
                        stats["short_fail"] += 1
                        continue

                    ver = payload[0]
                    msg = payload[1]
                    if ver != 1:
                        stats["unknown_msg"] += 1
                        continue
                    if msg != USBMSG_SLOW_SNAPSHOT:
                        stats["unknown_msg"] += 1
                        continue

                    seq_s, = struct.unpack_from("<I", payload, 2)
                    ts_ms, = struct.unpack_from("<I", payload, 6)
                    uid = uid_hex(payload[10:16])
                    addr = payload[16]
                    status, = struct.unpack_from("<H", payload, 17)
                    count = payload[19]

                    seen_uids.add(uid)

                    last = last_seq_by_uid.get(uid)
                    if last is not None and seq_s != last + 1:
                        stats["seq_gaps"] += 1
                        per_uid_stats(uid)["seq_gaps"] += 1
                    last_seq_by_uid[uid] = seq_s

                    off = 20
                    data_map = {}
                    for _ in range(count):
                        if off + 8 > len(payload):
                            break
                        ch = payload[off]; off += 1
                        vraw, = struct.unpack_from("<i", payload, off); off += 4
                        scale, = struct.unpack_from("<H", payload, off); off += 2
                        unit = payload[off]; off += 1
                        value = (vraw / scale) if scale else None
                        data_map[str(ch)] = {"raw": vraw, "scale": scale, "unit": unit, "value": value}

                    rec = {
                        "v": 1,
                        "bus": "slow",
                        "topic": "snapshot",
                        "uid": uid,
                        "addr": addr,
                        "seq": int(seq_s),
                        "ts_master_ms": int(ts_ms),
                        "status": int(status),
                        "pc_time": datetime.now().isoformat(timespec="milliseconds"),
                        "data": data_map
                    }

                    ndjson_path = slow_dir / f"slow_{uid}.ndjson"
                    if not ndjson_path.exists():
                        print(f"[NEW] slow ndjson: {ndjson_path.name}")
                    with open(ndjson_path, "a", encoding="utf-8") as f:
                        f.write(json.dumps(rec, ensure_ascii=False) + "\n")

                    stats["frames_ok"] += 1
                    per_uid_stats(uid)["ok"] += 1

                now = time.time()
                if (now - last_flush) * 1000.0 >= args.flush_ms:
                    last_flush = now
                    manifest["nodes"] = sorted(list(seen_uids))
                    manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
                    stats_path.write_text(json.dumps(stats, indent=2), encoding="utf-8")
                    print(f"[STAT] umx ok={good_umx} bad={bad_umx} fast_frames={good_fast_frames} slow_ok={stats['frames_ok']} slow_total={stats['frames_total']}")

    except KeyboardInterrupt:
        print("\n[STOP] Ctrl+C")
    finally:
        try:
            ser.close()
        except Exception:
            pass
        for f in (f_mic, f_vib, f_res):
            try:
                f.close()
            except Exception:
                pass
        manifest["nodes"] = sorted(list(seen_uids))
        manifest_path.write_text(json.dumps(manifest, indent=2), encoding="utf-8")
        stats_path.write_text(json.dumps(stats, indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
