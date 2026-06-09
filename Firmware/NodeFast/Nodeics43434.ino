/********************************************************************
 * FAST NODE: MIC (ICS43434) ESP32-S3  — RX GATING STREAM FIX
 *
 * - Phase A MGMT: COBS+CRC16, responds with HELLO_DESCR
 * - Phase B STREAM: fixed-length frames (no COBS)
 *   MIC: 48kHz, PCM16, 10ms block = 480 samples => 960 bytes payload
 *   Frame: PREAMBLE(16x0x55) + SOF(0xA5) + SEQ + PAYLOAD(960) + CRC16
 *
 * Fixes:
 *  - STREAM: disable RX most of the time (reduce RX ISR load when other nodes transmit)
 *  - Only open RX in short window near cycle boundary to catch RESYNC
 *  - RESYNC listen armed every RESYNC_EXPECT_MS (match master)
 *  - Tight slot late-window: never transmit late (avoid collision with VIB slot)
 *  - MGMT send: delimiter 0x00 ... 0x00 framing (robust)
 ********************************************************************/
struct PacketHeader;
#include <Arduino.h>
#include <Preferences.h>
#include <driver/i2s.h>
#include "driver/uart.h"

#define RS485_SERIAL       Serial2
#define RS485_BAUD         2000000
#define RS485_TX_PIN       17
#define RS485_RX_PIN       18

#define DBG_SERIAL         Serial
#define DBG_BAUD           115200

// ICS43434 I2S pins
#define I2S_PORT           I2S_NUM_0
#define I2S_WS_PIN         11
#define I2S_BCLK_PIN       9
#define I2S_DIN_PIN        10

// Audio config
static const uint32_t MIC_FS = 48000;
static const uint16_t MIC_BLOCK_MS = 10;
static const uint16_t MIC_SAMPLES_PER_BLOCK = (uint16_t)(MIC_FS * MIC_BLOCK_MS / 1000); // 480
static const uint16_t MIC_PAYLOAD_BYTES = MIC_SAMPLES_PER_BLOCK * 2; // 960

static uint32_t g_underrun = 0;

// Stream framing
static const uint8_t PREAMBLE_BYTE = 0x55;
static const uint8_t PREAMBLE_LEN  = 16;
static const uint8_t SOF_MIC       = 0xA5;
static const uint8_t SOF_RESYNC    = 0xA7;

#ifndef RESYNC_EXPECT_MS
#define RESYNC_EXPECT_MS       1000UL    // must match master RESYNC_PERIOD_MS (1s)
#endif
#define RESYNC_OPEN_PHASE_US   0UL     // open near cycle boundary
#define RESYNC_OPEN_WINDOW_US  3500UL    // keep RX open briefly to parse full resync

// Tight slot window for MIC (avoid stepping on VIB)
#define MIC_EARLY_US           400
#define MIC_LATE_US            200

// Serial2 is usually UART2 in Arduino-ESP32
static const uart_port_t RS_UART = UART_NUM_2;

// If 1: flush UART RX FIFO when RX is disabled (safer but can cost time/jitter)
#ifndef RS485_FLUSH_ON_DISABLE
#define RS485_FLUSH_ON_DISABLE 0
#endif

static inline void rs_rx_enable(bool en) {
  if (en) {
    uart_enable_rx_intr(RS_UART);
  } else {
    uart_disable_rx_intr(RS_UART);
    if (RS485_FLUSH_ON_DISABLE) {
      uart_flush_input(RS_UART); // drop everything accumulated
    }
  }
}


static const uint8_t MAGIC0 = 0xA5;
static const uint8_t MAGIC1 = 0x5A;
static const uint8_t VER    = 0x01;
static const uint8_t MASTER_ID = 250;
static const uint8_t BROADCAST_ID = 0;
static const uint8_t UNASSIGNED_ID = 0xFF;

#pragma pack(push, 1)
struct PacketHeader {
  uint8_t magic0, magic1, ver;
  uint8_t src, dst, type, flags;
  uint16_t seq;
  uint16_t len;
};
#pragma pack(pop)

enum MsgType : uint8_t {
  MSG_HELLO       = 0x03,
  MSG_ASSIGN_ID   = 0x04,
  MSG_ID_ACK      = 0x05,
  MSG_START_STREAM= 0x20,
  MSG_SYNC        = 0x70,
  MSG_DISC_OPEN   = 0x71,
  MSG_SCHEDULE    = 0x73,
};

enum DevType : uint8_t { DEV_MIC=1, DEV_VIB=2 };
enum StreamFmt : uint8_t { FMT_PCM16=1, FMT_XYZ16=2 };

static uint16_t crc16_ccitt_false(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t b = 0; b < 8; b++) {
      crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
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

static bool readCobsFrame(uint8_t* outPlain, size_t outCap, size_t* outLen, uint32_t timeoutMs) {
  uint32_t t0 = millis();
  static uint8_t rx[1024];
  static size_t rxLen = 0;

  while ((uint32_t)(millis() - t0) < timeoutMs) {
    while (RS485_SERIAL.available()) {
      uint8_t b = (uint8_t)RS485_SERIAL.read();
      if (b == 0x00) {
        if (rxLen == 0) continue;
        size_t plainLen = 0;
        bool ok = cobsDecode(rx, rxLen, outPlain, &plainLen);
        rxLen = 0;
        if (!ok || plainLen > outCap) return false;
        *outLen = plainLen;
        return true;
      }
      if (rxLen < sizeof(rx)) rx[rxLen++] = b;
      else rxLen = 0;
    }
    delay(1);
  }
  return false;
}

// delimiter framing for auto-dir robustness
static void sendMgmtPacket(uint8_t dst, uint8_t type, const uint8_t* payload, uint16_t plen) {
  uint8_t plain[512];
  uint8_t enc[700];
  if (sizeof(PacketHeader) + plen + 2 > sizeof(plain)) return;

  static uint16_t seq = 1;
  PacketHeader h{};
  h.magic0 = MAGIC0; h.magic1 = MAGIC1; h.ver = VER;
  h.src = (uint8_t)UNASSIGNED_ID; // keep as unassigned in mgmt frames
  h.dst = dst;
  h.type = type;
  h.flags = 0;
  h.seq = seq++;
  h.len = plen;

  memcpy(plain, &h, sizeof(h));
  if (plen) memcpy(plain + sizeof(h), payload, plen);
  uint16_t crc = crc16_ccitt_false(plain, sizeof(h) + plen);
  plain[sizeof(h) + plen + 0] = (uint8_t)(crc & 0xFF);
  plain[sizeof(h) + plen + 1] = (uint8_t)(crc >> 8);

  size_t encLen = cobsEncode(plain, sizeof(h) + plen + 2, enc);

  RS485_SERIAL.write((uint8_t)0x00);
  RS485_SERIAL.write(enc, encLen);
  RS485_SERIAL.write((uint8_t)0x00);
  RS485_SERIAL.flush();
}

static void get_uid48(uint8_t uid[6]) {
  uint64_t mac = ESP.getEfuseMac();
  uid[0] = (mac >> 40) & 0xFF;
  uid[1] = (mac >> 32) & 0xFF;
  uid[2] = (mac >> 24) & 0xFF;
  uid[3] = (mac >> 16) & 0xFF;
  uid[4] = (mac >>  8) & 0xFF;
  uid[5] = (mac >>  0) & 0xFF;
}

static uint8_t  g_uid[6];
static uint8_t  g_node_id = UNASSIGNED_ID;

// ===== FAST SCHEDULE (master-driven slots) =====
static uint32_t g_cycle_us = 10000;

struct Slot {
  uint32_t offset_us;
  uint8_t  period_cycles;
  uint8_t  sof;
};

static constexpr uint8_t MAX_SLOTS = 8;
static Slot     g_slots[MAX_SLOTS];
static uint8_t  g_slot_count = 0;

static inline void clear_slots() { g_slot_count = 0; }
static inline void add_slot(uint32_t off, uint8_t per, uint8_t sof) {
  if (g_slot_count >= MAX_SLOTS) return;
  if (per == 0) per = 1;
  g_slots[g_slot_count++] = { off, per, sof };
}

static bool     g_streaming = false;

static uint8_t  g_disc_epoch = 0;
static bool     g_hello_sent = false;
static bool     g_hello_pending = false;
static uint32_t g_hello_due_ms = 0;

static uint32_t g_epoch_start_us = 0;
static uint8_t  g_seq_stream = 0;

// RX gating / resync listen
static bool     g_resyncArmed = false;
static uint32_t g_lastListenMs = 0;

static const uint32_t RING_SAMPLES = MIC_SAMPLES_PER_BLOCK * 50; // 500ms
static int16_t micRing[RING_SAMPLES];
static volatile uint32_t ring_w = 0;
static volatile uint32_t ring_r = 0;

static inline uint32_t ring_count() {
  uint32_t w = ring_w, r = ring_r;
  return (w >= r) ? (w - r) : (RING_SAMPLES - (r - w));
}

static void ring_push(int16_t s) {
  uint32_t next = ring_w + 1;
  if (next >= RING_SAMPLES) next = 0;
  if (next == ring_r) {
    uint32_t r = ring_r + 1;
    if (r >= RING_SAMPLES) r = 0;
    ring_r = r;
  }
  micRing[ring_w] = s;
  ring_w = next;
}

static bool ring_pop_block(int16_t* out, uint32_t n) {
  if (ring_count() < n) return false;
  for (uint32_t i = 0; i < n; i++) {
    out[i] = micRing[ring_r];
    uint32_t r = ring_r + 1;
    if (r >= RING_SAMPLES) r = 0;
    ring_r = r;
  }
  return true;
}

static void i2s_init() {
  i2s_config_t cfg{};
  cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX);
  cfg.sample_rate = MIC_FS;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;
  cfg.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
  cfg.communication_format = I2S_COMM_FORMAT_I2S;
  cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  cfg.dma_buf_count = 6;
  cfg.dma_buf_len = 256;
  cfg.use_apll = true;
  cfg.tx_desc_auto_clear = false;
  cfg.fixed_mclk = MIC_FS * 256;

  i2s_pin_config_t pins{};
  pins.bck_io_num = I2S_BCLK_PIN;
  pins.ws_io_num = I2S_WS_PIN;
  pins.data_out_num = -1;
  pins.data_in_num = I2S_DIN_PIN;

  i2s_driver_install(I2S_PORT, &cfg, 0, nullptr);
  i2s_set_pin(I2S_PORT, &pins);
  i2s_zero_dma_buffer(I2S_PORT);
}

static void mic_capture_tick() {
  static int32_t buf32[256];

  for (int k = 0; k < 6; k++) {
    size_t bytesRead = 0;
    esp_err_t err = i2s_read(I2S_PORT, (void*)buf32, sizeof(buf32), &bytesRead, 0);
    if (err != ESP_OK || bytesRead == 0) break;

    uint32_t n = bytesRead / 4;
    for (uint32_t i = 0; i < n; i++) {
      int16_t s16 = (int16_t)(buf32[i] >> 14);
      ring_push(s16);
    }
  }
}

static void stream_send_mic_frame() {
  static int16_t block[MIC_SAMPLES_PER_BLOCK];
  static uint8_t frame[PREAMBLE_LEN + 1 + 1 + MIC_PAYLOAD_BYTES + 2];

  bool ok = ring_pop_block(block, MIC_SAMPLES_PER_BLOCK);
  if (!ok) {
    g_underrun++;
    memset(block, 0, sizeof(block));
  }

  for (uint8_t i = 0; i < PREAMBLE_LEN; i++) frame[i] = PREAMBLE_BYTE;
  size_t p = PREAMBLE_LEN;
  uint8_t seq = g_seq_stream++;
  frame[p++] = SOF_MIC;
  frame[p++] = seq;

  for (uint32_t i = 0; i < MIC_SAMPLES_PER_BLOCK; i++) {
    int16_t s = block[i];
    frame[p++] = (uint8_t)(s & 0xFF);
    frame[p++] = (uint8_t)((uint16_t)s >> 8);
  }

  uint16_t crc = crc16_ccitt_false(frame + PREAMBLE_LEN, 1 + 1 + MIC_PAYLOAD_BYTES);
  frame[p++] = (uint8_t)(crc & 0xFF);
  frame[p++] = (uint8_t)(crc >> 8);

  RS485_SERIAL.write(frame, p);
  RS485_SERIAL.flush();
}

static bool stream_try_parse_resync_budget(uint32_t budget_us) {
  static uint8_t preCnt = 0;
  static uint8_t st = 0;
  static uint8_t body[1 + 2 + 2 + 2]; // epoch + apply + cycle + crc
  static uint8_t idx = 0;

  uint32_t t0 = micros();
  while ((uint32_t)(micros() - t0) < budget_us) {
    int avail = RS485_SERIAL.available();
    if (avail <= 0) { delayMicroseconds(20); continue; }

    uint8_t b = (uint8_t)RS485_SERIAL.read();

    if (st == 0) {
      if (b == PREAMBLE_BYTE) { if (preCnt < PREAMBLE_LEN) preCnt++; }
      else preCnt = 0;

      if (preCnt == PREAMBLE_LEN) { st = 1; preCnt = 0; }
    } else if (st == 1) {
      if (b == SOF_RESYNC) { st = 2; idx = 0; }
      else st = 0;
    } else {
      body[idx++] = b;
      if (idx >= sizeof(body)) {
        uint8_t epoch = body[0];
        uint16_t apply_us = (uint16_t)body[1] | ((uint16_t)body[2] << 8);
        uint16_t cyc_us   = (uint16_t)body[3] | ((uint16_t)body[4] << 8);
        uint16_t rxcrc    = (uint16_t)body[5] | ((uint16_t)body[6] << 8);

        uint8_t tmp[6];
        tmp[0] = SOF_RESYNC;
        tmp[1] = epoch;
        tmp[2] = (uint8_t)(apply_us & 0xFF);
        tmp[3] = (uint8_t)(apply_us >> 8);
        tmp[4] = (uint8_t)(cyc_us & 0xFF);
        tmp[5] = (uint8_t)(cyc_us >> 8);

        uint16_t crc = crc16_ccitt_false(tmp, sizeof(tmp));
        st = 0;

        if (crc == rxcrc && cyc_us >= 5000 && cyc_us <= 20000) {
          g_cycle_us = cyc_us;
          g_epoch_start_us = (uint32_t)micros() + apply_us;

          (void)epoch;
          return true;
        }
      }
    }
  }
  return false;
}

static bool verify_and_extract(const uint8_t* plain, size_t plainLen, PacketHeader* h, const uint8_t** payload) {
  if (plainLen < sizeof(PacketHeader) + 2) return false;
  memcpy(h, plain, sizeof(PacketHeader));
  if (h->magic0 != MAGIC0 || h->magic1 != MAGIC1 || h->ver != VER) return false;
  uint16_t plen = h->len;
  if ((size_t)sizeof(PacketHeader) + plen + 2 != plainLen) return false;
  uint16_t rxcrc = (uint16_t)plain[plainLen - 2] | ((uint16_t)plain[plainLen - 1] << 8);
  uint16_t crc = crc16_ccitt_false(plain, plainLen - 2);
  if (crc != rxcrc) return false;
  *payload = plain + sizeof(PacketHeader);
  return true;
}

static void handle_mgmt_packet(const PacketHeader& h, const uint8_t* payload) {
  if (h.dst != BROADCAST_ID && h.dst != g_node_id && h.dst != UNASSIGNED_ID) return;

  if (h.type == MSG_DISC_OPEN) {
    uint8_t epoch = 1;
    uint8_t rand_max = 50;
    if (h.len >= 4) {
      epoch = payload[0];
      rand_max = payload[3];
    }
    if (rand_max < 10) rand_max = 10;
    if (rand_max > 250) rand_max = 250;

    if (epoch != g_disc_epoch) {
      g_disc_epoch = epoch;
      g_hello_sent = false;
      g_hello_pending = false;
    }
    if (!g_hello_sent && !g_hello_pending) {
      g_hello_due_ms = millis() + (uint32_t)random(0, (int)rand_max + 1);
      g_hello_pending = true;
    }
    return;
  }

  if (h.type == MSG_ASSIGN_ID) {
    if (h.len < 7) return;
    if (memcmp(payload, g_uid, 6) != 0) return;
    g_node_id = payload[6];
    DBG_SERIAL.printf("[MGMT] Assigned ID=%u\n", g_node_id);

    uint8_t ack[7];
    memcpy(ack, g_uid, 6);
    ack[6] = g_node_id;
    sendMgmtPacket(BROADCAST_ID, MSG_ID_ACK, ack, sizeof(ack));
    return;
  }

  if (h.type == MSG_SCHEDULE) {
    if (h.len < 2 + 4 + 1) return;

    uint16_t start_after_ms = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
    uint32_t cycle_us       = (uint32_t)payload[2] | ((uint32_t)payload[3] << 8) |
                              ((uint32_t)payload[4] << 16) | ((uint32_t)payload[5] << 24);
    uint8_t count           = payload[6];
    const uint8_t* p        = payload + 7;

    g_cycle_us = cycle_us;
    clear_slots();

    for (uint8_t i = 0; i < count; i++) {
      if ((size_t)(p - payload) + 6 + 1 + 4 + 1 + 1 > h.len) break;

      const uint8_t* uid = p;
      uint8_t  nid = p[6];
      uint32_t off = (uint32_t)p[7]  | ((uint32_t)p[8]  << 8) |
                     ((uint32_t)p[9]  << 16) | ((uint32_t)p[10] << 24);
      uint8_t  per = p[11];
      uint8_t  sof = p[12];

      if (memcmp(uid, g_uid, 6) == 0) {
        // Master-scheduled: each matching entry is one TX slot for this node.
        if (nid != UNASSIGNED_ID) g_node_id = nid;
        add_slot(off, per, sof);
      }

      p += (6 + 1 + 4 + 1 + 1);
    }

    DBG_SERIAL.printf("[MGMT] SCHED: id=%u cycle=%lu start_after=%u slots=%u\n",
                      g_node_id, (unsigned long)g_cycle_us, start_after_ms, (unsigned)g_slot_count);

    for (uint8_t i = 0; i < g_slot_count; i++) {
      DBG_SERIAL.printf("  [SLOT] #%u off=%lu per=%u sof=0x%02X\n",
                        (unsigned)i,
                        (unsigned long)g_slots[i].offset_us,
                        (unsigned)g_slots[i].period_cycles,
                        (unsigned)g_slots[i].sof);
    }

    g_epoch_start_us = (uint32_t)micros() + (uint32_t)start_after_ms * 1000UL;
    return;
  }

  if (h.type == MSG_START_STREAM) {
    if (g_epoch_start_us == 0) g_epoch_start_us = (uint32_t)micros() + 350000UL;
    g_streaming = true;
    DBG_SERIAL.println("[MGMT] START_STREAM -> streaming");

    // STREAM: disable RX by default
    rs_rx_enable(false);
    g_lastListenMs = millis();
    g_resyncArmed = false;
    return;
  }
}

static void mgmt_poll() {
  if (g_hello_pending && !g_hello_sent) {
    if ((int32_t)(millis() - g_hello_due_ms) >= 0) {
      uint8_t plh[12];
      memcpy(plh, g_uid, 6);
      plh[6] = DEV_MIC;
      uint16_t fs = (uint16_t)MIC_FS;
      plh[7] = (uint8_t)(fs & 0xFF);
      plh[8] = (uint8_t)(fs >> 8);
      plh[9] = FMT_PCM16;
      plh[10] = 1;
      plh[11] = MIC_BLOCK_MS;
      sendMgmtPacket(BROADCAST_ID, MSG_HELLO, plh, sizeof(plh));
      DBG_SERIAL.println("[MGMT] HELLO_DESCR sent");
      g_hello_sent = true;
      g_hello_pending = false;
    }
  }

  uint8_t plain[512];
  size_t plainLen = 0;
  for (int k = 0; k < 4; k++) {
    if (!readCobsFrame(plain, sizeof(plain), &plainLen, 3)) return;
    PacketHeader h{};
    const uint8_t* pl = nullptr;
    if (!verify_and_extract(plain, plainLen, &h, &pl)) continue;
    handle_mgmt_packet(h, pl);
  }
}

static void stream_tick() {
  uint32_t now = (uint32_t)micros();
  if (!g_streaming || g_epoch_start_us == 0) return;

  // RESYNC listen: open RX at the beginning of *every* cycle (master-only window)
  // This avoids "missed resync" when the CPU is busy and also continuously corrects drift.
  uint32_t nowMs = millis();
  uint32_t dt0 = (uint32_t)(now - g_epoch_start_us);
  uint32_t phase = (g_cycle_us > 0) ? (dt0 % g_cycle_us) : 0;

  if (phase < RESYNC_OPEN_WINDOW_US) {
    rs_rx_enable(true);

    // Small budget most of the time; if we haven't seen RESYNC for a while, listen longer.
    uint32_t budget = 250; // us
    if ((uint32_t)(nowMs - g_lastListenMs) > (RESYNC_EXPECT_MS * 3UL)) budget = RESYNC_OPEN_WINDOW_US;

    if (stream_try_parse_resync_budget(budget)) {
      g_lastListenMs = nowMs;
    }

    rs_rx_enable(false);
  }

  // ===== master-driven slot scheduler =====
  uint32_t e = (uint32_t)(now - g_epoch_start_us);
  if (g_cycle_us == 0) return;

  uint32_t cyc = e / g_cycle_us;

  // reset per-cycle sent mask
  static uint32_t lastCyc = 0xFFFFFFFF;
  static uint8_t  sentMask = 0;
  if (cyc != lastCyc) {
    lastCyc = cyc;
    sentMask = 0;
  }

  auto try_slot = [&](uint8_t bit, uint32_t target) {
    if (sentMask & bit) return;

    uint32_t tnow = (uint32_t)micros();
    int32_t d = (int32_t)(tnow - target);

    // too early (no busy wait)
    if (d < -MIC_EARLY_US) return;

    // too late -> skip (avoid collision with other slots/nodes)
    if (d > (int32_t)MIC_LATE_US) {
      sentMask |= bit;
      return;
    }

    sentMask |= bit;
    stream_send_mic_frame();
  };

  // Iterate all slots assigned by master
  for (uint8_t i = 0; i < g_slot_count && i < 8; i++) {
    uint8_t per = g_slots[i].period_cycles;
    if (per == 0) per = 1;
    if ((cyc % per) != 0) continue;

    uint32_t target = g_epoch_start_us + cyc * g_cycle_us + g_slots[i].offset_us;
    try_slot((uint8_t)(1u << i), target);
  }
}

void setup() {
  DBG_SERIAL.begin(DBG_BAUD);
  delay(100);
  DBG_SERIAL.println("NODE MIC boot");

  get_uid48(g_uid);
  DBG_SERIAL.printf("UID=%02X%02X%02X%02X%02X%02X\n", g_uid[0],g_uid[1],g_uid[2],g_uid[3],g_uid[4],g_uid[5]);

  RS485_SERIAL.begin(RS485_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  // MGMT needs RX
  rs_rx_enable(true);

  randomSeed((uint32_t)ESP.getEfuseMac());

  i2s_init();
  DBG_SERIAL.println("I2S init OK");
  DBG_SERIAL.println("[BOOT] waiting MGMT_OPEN...");
}

void loop() {
  mic_capture_tick();

  if (!g_streaming) {
    mgmt_poll();
    return;
  }

  stream_tick();
}
