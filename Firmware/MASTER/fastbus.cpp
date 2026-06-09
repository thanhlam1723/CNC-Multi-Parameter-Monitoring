#include "fastbus.h"
#include "usbmux.h"

namespace fastbus {
  void fast_init();   
  void fast_tick();   
  bool is_streaming();
}

#ifndef RS485_GUARD_US
#define RS485_GUARD_US 200
#endif

#define RS485_SERIAL       Serial2
#define RS485_BAUD         2000000
#define RS485_TX_PIN       17
#define RS485_RX_PIN       18

#define DBG_SERIAL         DBG
#define DBG_BAUD           115200

#define DATA_SERIAL        Serial   
#define DATA_BAUD          2000000  

static const uint16_t DISCOVERY_WINDOW_MS   = 3000;
static const uint16_t DISC_OPEN_PERIOD_MS   = 200;
static const uint8_t  NODE_BACKOFF_MAX_MS   = 200;
static const uint16_t ACK_WAIT_MS           = 900;

static const uint32_t CYCLE_US              = 20000;     
static const uint16_t SYNC_WINDOW_US         = 2000;      
static const uint16_t START_AFTER_MS        = 600;       

static const uint32_t RESYNC_PERIOD_MS      = 1000;  
static const uint16_t RESYNC_APPLY_DELAY_US = 1000;

static const uint8_t SOF_MIC    = 0xA5;
static const uint8_t SOF_VIB    = 0xA6;
static const uint8_t SOF_RESYNC = 0xA7;

static const uint8_t PREAMBLE_BYTE       = 0x55;
static const uint8_t PREAMBLE_LEN_RESYNC = 16;

static const uint8_t MAGIC0 = 0xA5;
static const uint8_t MAGIC1 = 0x5A;
static const uint8_t VER    = 0x01;

static const uint8_t MASTER_ID     = 250;
static const uint8_t BROADCAST_ID  = 0x00;
static const uint8_t UNASSIGNED_ID = 0xFF;

static uint16_t crc16_ccitt_false(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t b = 0; b < 8; b++) {
      crc = (crc & 0x8000) ? (uint16_t)(((uint16_t)(crc << 1)) ^ 0x1021) : (uint16_t)(crc << 1);
    }
  }
  return crc;
}

static size_t cobsEncode(const uint8_t* input, size_t length, uint8_t* output) {
  uint8_t* out0 = output;
  const uint8_t* end = input + length;
  uint8_t* code_ptr = output++;
  uint8_t code = 1;

  while (input < end) {
    if (*input == 0) {
      *code_ptr = code;
      code_ptr = output++;
      code = 1;
      input++;
    } else {
      *output++ = *input++;
      code++;
      if (code == 0xFF) {
        *code_ptr = code;
        code_ptr = output++;
        code = 1;
      }
    }
  }
  *code_ptr = code;
  return (size_t)(output - out0);
}

static bool cobsDecode(const uint8_t* input, size_t length, uint8_t* output, size_t* outLen) {
  size_t in = 0, out = 0;
  while (in < length) {
    uint8_t code = input[in++];
    if (code == 0) return false;
    for (uint8_t i = 1; i < code; i++) {
      if (in >= length) return false;
      output[out++] = input[in++];
    }
    if (code < 0xFF && in < length) output[out++] = 0;
  }
  *outLen = out;
  return true;
}

static void uidToStr(const uint8_t uid[6], char* out, size_t outCap) {
  snprintf(out, outCap, "%02X%02X%02X%02X%02X%02X", uid[0], uid[1], uid[2], uid[3], uid[4], uid[5]);
}
static bool uidEqual(const uint8_t a[6], const uint8_t b[6]) {
  for (int i = 0; i < 6; i++) if (a[i] != b[i]) return false;
  return true;
}

static uint16_t g_mgmtSeq = 1;

static void sendMgmtPacket(uint8_t dst, uint8_t type, const uint8_t* payload, uint16_t len) {
  uint8_t plain[512];
  if (len > 400) return;

  PacketHeader h;
  h.magic0 = MAGIC0; h.magic1 = MAGIC1; h.ver = VER;
  h.src = MASTER_ID; h.dst = dst;
  h.type = type; h.flags = 0;
  h.seq = g_mgmtSeq++;
  h.len = len;

  size_t p = 0;
  memcpy(plain + p, &h, sizeof(h)); p += sizeof(h);
  if (len && payload) { memcpy(plain + p, payload, len); p += len; }

  uint16_t crc = crc16_ccitt_false(plain, p);
  plain[p++] = (uint8_t)(crc & 0xFF);
  plain[p++] = (uint8_t)(crc >> 8);

  uint8_t enc[600];
  size_t encLen = cobsEncode(plain, p, enc);

  RS485_SERIAL.write((uint8_t)0x00);
  RS485_SERIAL.write(enc, encLen);
  RS485_SERIAL.write((uint8_t)0x00);
  RS485_SERIAL.flush();
}

static bool readMgmtPacket(uint8_t* outPlain, size_t outCap, size_t* outLen, uint32_t timeoutMs) {
  uint32_t t0 = millis();
  static uint8_t rx[700];
  static size_t rxLen = 0;

  while ((uint32_t)(millis() - t0) < timeoutMs) {
    while (RS485_SERIAL.available()) {
      uint8_t b = (uint8_t)RS485_SERIAL.read();
      if (b == 0x00) {
        if (rxLen == 0) continue;
        size_t plainLen = 0;
        bool ok = cobsDecode(rx, rxLen, outPlain, &plainLen);
        rxLen = 0;
        if (!ok) continue;
        if (plainLen > outCap) continue;
        *outLen = plainLen;
        return true;
      } else {
        if (rxLen < sizeof(rx)) rx[rxLen++] = b;
        else rxLen = 0;
      }
    }
    delay(1);
  }
  return false;
}

static bool parseAndCheckMgmt(const uint8_t* plain, size_t plainLen, PacketHeader* outH, const uint8_t** outPayload) {
  if (plainLen < sizeof(PacketHeader) + 2) return false;
  PacketHeader h;
  memcpy(&h, plain, sizeof(h));
  if (h.magic0 != MAGIC0 || h.magic1 != MAGIC1 || h.ver != VER) return false;
  if (h.dst != MASTER_ID && h.dst != BROADCAST_ID) return false;

  const uint16_t len = h.len;
  if (sizeof(PacketHeader) + len + 2 != plainLen) return false;

  uint16_t got  = (uint16_t)plain[plainLen - 2] | ((uint16_t)plain[plainLen - 1] << 8);
  uint16_t calc = crc16_ccitt_false(plain, plainLen - 2);
  if (got != calc) return false;

  *outH = h;
  *outPayload = plain + sizeof(PacketHeader);
  return true;
}

static const uint8_t  STREAM_PREAMBLE_LEN = 16;
static const uint16_t STREAM_CRC_BYTES    = 2;
static const uint16_t STREAM_HDR_BYTES    = 1  + 1 ;
static const uint16_t AUTO_GUARD_US       = 1200;   

static uint32_t payload_bytes_for(const NodeInfo& n) {
  const uint32_t samples = (uint32_t)n.fs * (uint32_t)n.block_ms / 1000UL;
  if (n.dev == DEV_MIC) return samples * 2UL;     
  if (n.dev == DEV_VIB) return samples * 6UL;     
  return 0;
}
static uint32_t frame_bytes_for(const NodeInfo& n) {
  return (uint32_t)STREAM_PREAMBLE_LEN + (uint32_t)STREAM_HDR_BYTES + payload_bytes_for(n) + (uint32_t)STREAM_CRC_BYTES;
}
static uint32_t tx_time_us_for_bytes(uint32_t frame_bytes) {
  const uint64_t bits = (uint64_t)frame_bytes * 10ULL; 
  const uint64_t us = (bits * 1000000ULL + (RS485_BAUD - 1)) / (uint64_t)RS485_BAUD;
  return (uint32_t)us;
}

static NodeInfo g_nodes[8];
static uint8_t g_nodeCount = 0;

static NodeInfo* find_node_by_id(uint8_t id) {
  for (uint8_t i = 0; i < g_nodeCount; i++) {
    if (g_nodes[i].assigned_id == id) return &g_nodes[i];
  }
  return nullptr;
}

static int findNodeByUid(const uint8_t uid[6]) {
  for (uint8_t i = 0; i < g_nodeCount; i++) if (uidEqual(g_nodes[i].uid, uid)) return (int)i;
  return -1;
}
static void upsertNodeFromHelloDescr(const HelloDescr& d) {
  int idx = findNodeByUid(d.uid);
  if (idx < 0) {
    if (g_nodeCount >= (uint8_t)(sizeof(g_nodes)/sizeof(g_nodes[0]))) return;
    idx = g_nodeCount++;
    memcpy(g_nodes[idx].uid, d.uid, 6);
    g_nodes[idx].assigned_id = 0;
    g_nodes[idx].id_acked = false;
    g_nodes[idx].selected = false;
  }
  g_nodes[idx].dev = (DevType)d.dev_type;
  g_nodes[idx].fs = d.fs_hz;
  g_nodes[idx].fmt = (StreamFmt)d.fmt;
  g_nodes[idx].ch_axes = d.ch_or_axes;
  g_nodes[idx].block_ms = d.block_ms;
}

static void printNodeTable() {
  DBG_SERIAL.printf("[MGMT] nodes=%u\n", g_nodeCount);
  for (uint8_t i = 0; i < g_nodeCount; i++) {
    char u[20]; uidToStr(g_nodes[i].uid, u, sizeof(u));
    DBG_SERIAL.printf("  #%u uid=%s dev=%s fs=%u fmt=%u block=%ums\n",
      i, u,
      (g_nodes[i].dev==DEV_MIC?"MIC":(g_nodes[i].dev==DEV_VIB?"VIB":"UNK")),
      g_nodes[i].fs, (unsigned)g_nodes[i].fmt, (unsigned)g_nodes[i].block_ms);
  }
  DBG_SERIAL.flush();
}

static uint8_t  g_resyncPend[32];   
static uint8_t  g_resyncPendLen = 0;

static void sendResyncRaw(uint8_t epoch, uint16_t apply_delay_us, uint16_t cycle_us) {
  uint8_t buf[32];
  size_t p = 0;
  for (uint8_t i = 0; i < PREAMBLE_LEN_RESYNC; i++) buf[p++] = PREAMBLE_BYTE;
  buf[p++] = SOF_RESYNC;
  buf[p++] = epoch;
  buf[p++] = (uint8_t)(apply_delay_us & 0xFF);
  buf[p++] = (uint8_t)(apply_delay_us >> 8);
  buf[p++] = (uint8_t)(cycle_us & 0xFF);
  buf[p++] = (uint8_t)(cycle_us >> 8);

  uint16_t crc = crc16_ccitt_false(buf + PREAMBLE_LEN_RESYNC, p - PREAMBLE_LEN_RESYNC);
  buf[p++] = (uint8_t)(crc & 0xFF);
  buf[p++] = (uint8_t)(crc >> 8);

  RS485_SERIAL.write(buf, p);
  RS485_SERIAL.flush();

  if (p <= sizeof(g_resyncPend)) {
    memcpy(g_resyncPend, buf, p);
    g_resyncPendLen = (uint8_t)p;
  }
}

static bool     g_streaming     = false;
static uint32_t g_streamStartUs = 0;

static uint32_t g_lastResyncMs  = 0;
static bool     g_resyncArmed   = false;
static uint8_t  g_epoch         = 0;

static void runPhaseA() {
  DBG_SERIAL.println("[BOOT] FAST master starting Phase A (DISCOVERY)");

  uint32_t t0 = millis();
  uint32_t nextBcast = 0;

  uint8_t disc_epoch = (uint8_t)((micros() & 0xFF) ^ 0x5A);
  if (disc_epoch == 0) disc_epoch = 1;

  uint8_t openPayload[4];
  uint16_t win_ms = DISCOVERY_WINDOW_MS;
  openPayload[0] = disc_epoch;
  openPayload[1] = (uint8_t)(win_ms & 0xFF);
  openPayload[2] = (uint8_t)(win_ms >> 8);
  openPayload[3] = NODE_BACKOFF_MAX_MS;

  uint8_t plain[512];
  size_t plainLen = 0;

  while ((uint32_t)(millis() - t0) < DISCOVERY_WINDOW_MS) {
    if ((uint32_t)(millis() - nextBcast) >= DISC_OPEN_PERIOD_MS) {
      nextBcast = millis();
      sendMgmtPacket(BROADCAST_ID, MSG_DISC_OPEN, openPayload, sizeof(openPayload));
    }

    if (readMgmtPacket(plain, sizeof(plain), &plainLen, 10)) {
      PacketHeader h;
      const uint8_t* payload = nullptr;
      if (!parseAndCheckMgmt(plain, plainLen, &h, &payload)) continue;

      if (h.type == MSG_HELLO) {
        if (h.len == sizeof(HelloDescr)) {
          HelloDescr d;
          memcpy(&d, payload, sizeof(d));
          upsertNodeFromHelloDescr(d);
        } else if (h.len >= 7) {

          HelloDescr d{};
          memcpy(d.uid, payload, 6);
          uint8_t caps = payload[6];
          d.dev_type = (caps & 0x01) ? DEV_MIC : DEV_VIB;
          d.fs_hz    = (d.dev_type == DEV_MIC) ? 48000 : 1600;
          d.fmt      = (d.dev_type == DEV_MIC) ? FMT_PCM16 : FMT_XYZ16;
          d.ch_or_axes = (d.dev_type == DEV_MIC) ? 1 : 3;
          d.block_ms = (d.dev_type == DEV_MIC) ? 10 : 20;
          upsertNodeFromHelloDescr(d);
        }
      }
    }
  }

  printNodeTable();

  for (uint8_t i = 0; i < g_nodeCount; i++) {
    g_nodes[i].selected = true;
    g_nodes[i].assigned_id = 0;
    g_nodes[i].id_acked = false;
  }

  uint8_t nextId = 1;

  auto pump_rx_update_nodes = [&](uint32_t timeout_ms) {
    uint8_t plain[256];
    size_t  plainLen = 0;
    uint32_t t0 = millis();
    while ((uint32_t)(millis() - t0) < timeout_ms) {
      if (readMgmtPacket(plain, sizeof(plain), &plainLen, 10)) {
        PacketHeader h;
        const uint8_t* payload = nullptr;

        if (!parseAndCheckMgmt(plain, plainLen, &h, &payload)) continue;

        if (h.type == MSG_ID_ACK && h.len >= 7) {
          const uint8_t* uid = payload;
          uint8_t myid = payload[6];
          int nidx = findNodeByUid(uid);
          if (nidx >= 0 && g_nodes[nidx].assigned_id == myid) g_nodes[nidx].id_acked = true;
        } else if (h.type == MSG_HELLO && h.len == sizeof(HelloDescr)) {
          HelloDescr d; memcpy(&d, payload, sizeof(d));
          upsertNodeFromHelloDescr(d);
        }
      }
    }
  };

  auto assign_one_with_retry = [&](int nidx, uint8_t id) -> bool {
    g_nodes[nidx].assigned_id = id;
    g_nodes[nidx].id_acked = false;

    uint8_t pl[7];
    memcpy(pl, g_nodes[nidx].uid, 6);
    pl[6] = id;

    for (uint8_t attempt = 0; attempt < 4; attempt++) {

      while (RS485_SERIAL.available()) RS485_SERIAL.read();

      sendMgmtPacket(BROADCAST_ID, MSG_ASSIGN_ID, pl, sizeof(pl));

      delay(2);

      pump_rx_update_nodes(70);
      if (g_nodes[nidx].id_acked) return true;

      delay(15);
    }
    return false;
  };

  int micIdx = -1;
  for (uint8_t i = 0; i < g_nodeCount; i++) {
    if (g_nodes[i].selected && g_nodes[i].dev == DEV_MIC) { micIdx = (int)i; break; }
  }
  if (micIdx >= 0) {
    if (!assign_one_with_retry(micIdx, nextId)) {
      char u[20]; uidToStr(g_nodes[micIdx].uid, u, sizeof(u));
      DBG_SERIAL.printf("[WARN] MIC %s did not ACK ASSIGN_ID after retries.\n", u);
    }
    nextId++;
  }

  for (uint8_t i = 0; i < g_nodeCount; i++) {
    if (!g_nodes[i].selected) continue;
    if ((int)i == micIdx) continue;
    if (!assign_one_with_retry((int)i, nextId)) {
      char u[20]; uidToStr(g_nodes[i].uid, u, sizeof(u));
      DBG_SERIAL.printf("[WARN] Node %s did not ACK ASSIGN_ID after retries.\n", u);
    }
    nextId++;
  }

  pump_rx_update_nodes(50);

  const uint32_t MARGIN_US = 200;                 
  const uint32_t GUARD_US  = AUTO_GUARD_US;       
  const uint32_t GAP_US    = RS485_GUARD_US;      

  SchedEntry entries[16];
  uint8_t count = 0;

  auto append_entry = [&](const NodeInfo& n, uint32_t off) -> bool {
    if (count >= (uint8_t)(sizeof(entries)/sizeof(entries[0]))) return false;

    const uint32_t fb    = frame_bytes_for(n);
    const uint32_t tx_us = tx_time_us_for_bytes(fb);

    if (off + tx_us + GUARD_US > (uint32_t)CYCLE_US - MARGIN_US) {
      char u[20]; uidToStr(n.uid, u, sizeof(u));
      DBG_SERIAL.printf("[WARN] TDMA overflow: %s dev=%s off=%lu tx=%lu guard=%lu cycle=%u -> skip\n",
                        u, (n.dev==DEV_MIC?"MIC":(n.dev==DEV_VIB?"VIB":"UNK")),
                        (unsigned long)off, (unsigned long)tx_us, (unsigned long)GUARD_US, (unsigned)CYCLE_US);
      return false;
    }

    SchedEntry e{};
    memcpy(e.uid, n.uid, 6);
    e.id = n.assigned_id;
    e.offset_us = off;
    e.period_cycles = 1; 
    e.stream_sof = (n.dev == DEV_MIC) ? SOF_MIC : SOF_VIB;
    entries[count++] = e;

    DBG_SERIAL.printf("[SCHED] dev=%s id=%u frame=%luB tx=%luus off=%luus\n",
      (n.dev==DEV_MIC?"MIC":(n.dev==DEV_VIB?"VIB":"UNK")),
      (unsigned)n.assigned_id,
      (unsigned long)fb, (unsigned long)tx_us,
      (unsigned long)off);

    return true;
  };

  auto find_first = [&](DevType devWanted) -> NodeInfo* {
    for (uint8_t k = 0; k < g_nodeCount; k++) {
      if (!g_nodes[k].selected || g_nodes[k].assigned_id == 0) continue;
      if (g_nodes[k].dev != devWanted) continue;
      return &g_nodes[k];
    }
    return nullptr;
  };

  NodeInfo* mic = find_first(DEV_MIC);
  NodeInfo* vib = find_first(DEV_VIB);

  uint32_t off = (uint32_t)SYNC_WINDOW_US + MARGIN_US;

  if (mic) {
    const uint32_t mic_fb = frame_bytes_for(*mic);
    const uint32_t mic_tx = tx_time_us_for_bytes(mic_fb);

    if (append_entry(*mic, off)) {
      off += mic_tx + GUARD_US + GAP_US;
    }
  }

  if (vib) {
    const uint32_t vib_fb = frame_bytes_for(*vib);
    const uint32_t vib_tx = tx_time_us_for_bytes(vib_fb);

    if (append_entry(*vib, off)) {
      off += vib_tx + GUARD_US + GAP_US;
    }
  }

  if (mic) {
    const uint32_t mic_fb = frame_bytes_for(*mic);
    const uint32_t mic_tx = tx_time_us_for_bytes(mic_fb);

    uint32_t mic2_off = (uint32_t)SYNC_WINDOW_US + MARGIN_US + 10000; 

    if (mic2_off < off) mic2_off = off;

    if (mic2_off + mic_tx + GUARD_US <= (uint32_t)CYCLE_US - MARGIN_US) {
      append_entry(*mic, mic2_off);
    } else {
      DBG_SERIAL.println("[WARN] MIC#2 does not fit in this cycle; scheduled MIC only once (50fps).");
    }
  }

  if (count == 0) {

    DBG_SERIAL.println("[WARN] No nodes selected for schedule.");
  }

  for (uint8_t i = 0; i < count; i++) {
    NodeInfo* np = find_node_by_id(entries[i].id);
    if (!np) continue;
    const uint32_t fb = frame_bytes_for(*np);
    const uint32_t tx_us = tx_time_us_for_bytes(fb);
    DBG_SERIAL.printf("[SCHED] #%u dev=%s id=%u payload=%luB frame=%luB tx=%luus off=%luus per=%u sof=0x%02X\n",
      (unsigned)i,
      (np->dev==DEV_MIC?"MIC":(np->dev==DEV_VIB?"VIB":"UNK")),
      (unsigned)entries[i].id,
      (unsigned long)payload_bytes_for(*np),
      (unsigned long)fb,
      (unsigned long)tx_us,
      (unsigned long)entries[i].offset_us,
      (unsigned)entries[i].period_cycles,
      (unsigned)entries[i].stream_sof
    );
  }

  DBG_SERIAL.flush();

  static const size_t SCHED_ENTRY_WIRE = 13;
  static const uint8_t MAX_SCHED_ENTRIES = 16;
  if (count > MAX_SCHED_ENTRIES) count = MAX_SCHED_ENTRIES;
  uint8_t schedPayload[2 + 4 + 1 + MAX_SCHED_ENTRIES * SCHED_ENTRY_WIRE];
  size_t sp = 0;

  uint16_t startAfter = START_AFTER_MS;
  schedPayload[sp++] = (uint8_t)(startAfter & 0xFF);
  schedPayload[sp++] = (uint8_t)(startAfter >> 8);

  uint32_t cyc = CYCLE_US;
  schedPayload[sp++] = (uint8_t)(cyc & 0xFF);
  schedPayload[sp++] = (uint8_t)(cyc >> 8);
  schedPayload[sp++] = (uint8_t)(cyc >> 16);
  schedPayload[sp++] = (uint8_t)(cyc >> 24);

  schedPayload[sp++] = count;
  for (uint8_t i = 0; i < count; i++) {

    memcpy(schedPayload + sp, entries[i].uid, 6); sp += 6;
    schedPayload[sp++] = entries[i].id;
    uint32_t off = entries[i].offset_us;
    schedPayload[sp++] = (uint8_t)(off & 0xFF);
    schedPayload[sp++] = (uint8_t)((off >> 8) & 0xFF);
    schedPayload[sp++] = (uint8_t)((off >> 16) & 0xFF);
    schedPayload[sp++] = (uint8_t)((off >> 24) & 0xFF);
    schedPayload[sp++] = entries[i].period_cycles;
    schedPayload[sp++] = entries[i].stream_sof;
  }

  sendMgmtPacket(BROADCAST_ID, MSG_SCHEDULE, schedPayload, (uint16_t)sp);
  delay(50);
  sendMgmtPacket(BROADCAST_ID, MSG_START_STREAM, nullptr, 0);

  DBG_SERIAL.println("[MGMT] Phase A complete. STREAM is on USB-CDC now (binary).");
  DBG_SERIAL.flush();

  while (RS485_SERIAL.available()) (void)RS485_SERIAL.read();

  g_streaming = true;
  g_streamStartUs = micros() + (uint32_t)START_AFTER_MS * 1000UL;
  g_lastResyncMs = millis() - RESYNC_PERIOD_MS; 

  g_resyncArmed = false;
  g_epoch = 0;
}

static void runPhaseB() {

  static uint8_t  rb[131072];
  static uint32_t w = 0, r = 0;

  auto rb_count = [&]() -> uint32_t {
    return (w >= r) ? (w - r) : (uint32_t)(sizeof(rb) - (r - w));
  };
  auto rb_space = [&]() -> uint32_t {
    return (uint32_t)(sizeof(rb) - 1 - rb_count());
  };

  static uint32_t drop_bytes  = 0;
  static uint32_t drop_events = 0;

  const uint32_t nowUs = micros();
  const uint32_t nowMs = millis();

  bool in_stream = ((int32_t)(nowUs - g_streamStartUs) >= 0);
  uint32_t dt_us = in_stream ? (uint32_t)(nowUs - g_streamStartUs) : 0;
  uint32_t phase = in_stream ? (dt_us % CYCLE_US) : 0;
  uint32_t cyc   = in_stream ? (dt_us / CYCLE_US) : 0;

  if (g_resyncPendLen) {

    if (usbmux::fast_chunk(g_resyncPend, (uint16_t)g_resyncPendLen)) {
      g_resyncPendLen = 0;
    }
  }

  if (RESYNC_PERIOD_MS > 0 && in_stream) {
    if (!g_resyncArmed && (uint32_t)(nowMs - g_lastResyncMs) >= RESYNC_PERIOD_MS) {
      g_resyncArmed = true;
    }

    static uint32_t lastSentCyc = 0xFFFFFFFF;
    if (g_resyncArmed && phase < SYNC_WINDOW_US && cyc != lastSentCyc) {

      if (true) {
        g_epoch++;
        sendResyncRaw(g_epoch, RESYNC_APPLY_DELAY_US, (uint16_t)CYCLE_US);
        g_lastResyncMs = nowMs;
        g_resyncArmed  = false;
        lastSentCyc    = cyc;
      }
    }
  }

  if (in_stream && phase < SYNC_WINDOW_US) {

    uint32_t cnt = rb_count();
    if (cnt) {
      int can = (int)Serial.availableForWrite(); 
      if (can > 0) {
        uint32_t sendn = cnt;
        if (sendn > (uint32_t)can) sendn = (uint32_t)can;
        if (sendn > 4096) sendn = 4096;
        uint32_t head = (w >= r) ? (w - r) : (uint32_t)(sizeof(rb) - r);
        if (sendn > head) sendn = head;
        usbmux::fast_chunk(&rb[r], (uint16_t)sendn);
        r = (r + sendn) % (uint32_t)sizeof(rb);
      }
    }
    return;
  }

  int avail = RS485_SERIAL.available();
  while (avail > 0) {
    uint32_t space = rb_space();

    if (space == 0) {
      drop_events++;
      while (RS485_SERIAL.available()) { (void)RS485_SERIAL.read(); drop_bytes++; }
      break;
    }

    uint32_t chunk = space;
    if (chunk > 2048) chunk = 2048;

    uint32_t tail = (w >= r) ? (uint32_t)(sizeof(rb) - w) : (uint32_t)(r - w - 1);
    if (chunk > tail) chunk = tail;

    int n = RS485_SERIAL.readBytes(&rb[w], (size_t)chunk);
    if (n <= 0) break;

    w = (w + (uint32_t)n) % (uint32_t)sizeof(rb);
    avail = RS485_SERIAL.available();
  }

  uint32_t cnt = rb_count();
  if (cnt == 0) return;

  int can = (int)Serial.availableForWrite(); 
  if (can <= 0) return;

  uint32_t sendn = cnt;
  if (sendn > (uint32_t)can) sendn = (uint32_t)can;
  if (sendn > 4096) sendn = 4096;

  uint32_t head = (w >= r) ? (w - r) : (uint32_t)(sizeof(rb) - r);
  if (sendn > head) sendn = head;

  usbmux::fast_chunk(&rb[r], (uint16_t)sendn);
  r = (r + sendn) % (uint32_t)sizeof(rb);

  static uint32_t lastDbg = 0;
  if ((uint32_t)(nowMs - lastDbg) > 2000) {
    lastDbg = nowMs;
    DBG_SERIAL.printf("[STAT] rb_cnt=%lu rb_space=%lu drop_ev=%lu drop_b=%lu phase=%lu\n",
                      (unsigned long)rb_count(),
                      (unsigned long)rb_space(),
                      (unsigned long)drop_events,
                      (unsigned long)drop_bytes,
                      (unsigned long)phase);
  }
}

namespace fastbus {
  void fast_init() {

  DBG_SERIAL.println("[FAST] init");

  RS485_SERIAL.begin(RS485_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  RS485_SERIAL.setRxBufferSize(16384);
  delay(50);

  runPhaseA(); 
  }
  void fast_tick() {

  if (g_streaming) runPhaseB();
  }
  bool is_streaming() { return g_streaming; }
}
