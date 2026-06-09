#ifndef RS485_GUARD_US
#define RS485_GUARD_US 200
#endif
/********************************************************************
 * FAST NODE: VIB (ADXL345) ESP32-WROOM / ESP32  — RX GATING FIX
 *
 * Fixes:
 *  - During STREAM: RX is DISABLED most of the time (reduce ISR/load)
 *  - Only OPEN RX in a small window near cycle boundary to catch RESYNC
 *  - RESYNC listen is ARMED every RESYNC_EXPECT_MS (match master)
 *  - Add LATE_US window: if too late => SKIP this cycle (avoid collision)
 *  - MGMT COBS framing: add leading 0x00 delimiter (robust w/ auto-dir)
 ********************************************************************/

#include <Arduino.h>
#include <Preferences.h>
#include <SPI.h>
#include "driver/uart.h"

#define RS485_SERIAL       Serial2
#define RS485_BAUD         2000000
#define RS485_TX_PIN       17
#define RS485_RX_PIN       16

#define DBG_SERIAL         Serial
#define DBG_BAUD           115200

// ADXL345 SPI pins (update to match your wiring)
#define ADXL_CS_PIN        5
#define ADXL_SCK_PIN       18
#define ADXL_MISO_PIN      19
#define ADXL_MOSI_PIN      23

#ifndef ADXL_ODR_HZ
#define ADXL_ODR_HZ        1600
#endif

// Protocol stream constants
#define STREAM_PREAMBLE_LEN    16
#define SOF_VIB                0xA6
#define SOF_RESYNC             0xA7

// Block policy
#define CYCLE_US               20000
#define VIB_BLOCK_MS           20

#define VIB_SAMPLES_PER_BLOCK  ((ADXL_ODR_HZ * VIB_BLOCK_MS) / 1000)
#define VIB_PAYLOAD_BYTES      (VIB_SAMPLES_PER_BLOCK * 6)

// IMPORTANT: set this to match master RESYNC_PERIOD_MS (e.g. 30000)
#ifndef RESYNC_EXPECT_MS
#define RESYNC_EXPECT_MS       1000UL    // must match master RESYNC_PERIOD_MS (1s)
#endif

// Open RX only near cycle boundary (master sends resync at phase < ~200us)
#define RESYNC_OPEN_PHASE_US   0UL
// Keep RX open briefly to capture full resync frame + jitter margin
#define RESYNC_OPEN_WINDOW_US  3500UL

// Late window for VIB slot (prevent late TX collision)
#define VIB_LATE_US            300UL

// Which UART is Serial2 on ESP32 Arduino? Usually UART2.
static const uart_port_t RS_UART = UART_NUM_2;

static const uint8_t MAGIC0 = 0xA5, MAGIC1 = 0x5A, VER = 0x01;
static const uint8_t MASTER_ID = 250;
static const uint8_t BROADCAST_ID = 0;

#pragma pack(push, 1)
struct PacketHeader {
  uint8_t magic0, magic1, ver;
  uint8_t src, dst, type, flags;
  uint16_t seq;
  uint16_t len;
};
#pragma pack(pop)

enum MsgType : uint8_t {
  MSG_HELLO        = 0x03,
  MSG_ASSIGN_ID    = 0x04,
  MSG_ID_ACK       = 0x05,
  MSG_START_STREAM = 0x20,
  MSG_SYNC         = 0x70,
  MSG_DISC_OPEN    = 0x71,
  MSG_SCHEDULE     = 0x73,
};

enum DevType : uint8_t { DEV_VIB = 2 };
enum StreamFmt : uint8_t { FMT_XYZ16 = 2 };

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

// incremental CRC helper
static uint16_t crc16_update(uint16_t crc, const uint8_t* data, size_t len) {
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

static bool readCobsFrameRS(uint8_t* outPlain, size_t outCap, size_t* outLen, uint32_t timeoutMs) {
  uint32_t t0 = millis();
  static uint8_t rx[512];
  static size_t rxLen = 0;

  while ((uint32_t)(millis() - t0) < timeoutMs) {
    while (RS485_SERIAL.available()) {
      uint8_t b = (uint8_t)RS485_SERIAL.read();
      if (b == 0x00) {
        if (rxLen == 0) continue;
        size_t plainLen = 0;
        bool ok = cobsDecode(rx, rxLen, outPlain, &plainLen);
        rxLen = 0;
        if (!ok) return false;
        if (plainLen > outCap) return false;
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

static void sendCobsPacket(uint8_t dst, uint8_t type, const uint8_t* payload, uint16_t plen) {
  uint8_t plain[320];
  PacketHeader h;
  h.magic0 = MAGIC0; h.magic1 = MAGIC1; h.ver = VER;
  h.src = 0xFF; // temporary
  h.dst = dst;
  h.type = type;
  h.flags = 0;
  static uint16_t seq = 1;
  h.seq = seq++;
  h.len = plen;

  size_t off = 0;
  memcpy(plain + off, &h, sizeof(h)); off += sizeof(h);
  if (plen && payload) { memcpy(plain + off, payload, plen); off += plen; }
  uint16_t crc = crc16_ccitt_false(plain, off);
  plain[off++] = (uint8_t)(crc & 0xFF);
  plain[off++] = (uint8_t)(crc >> 8);

  uint8_t enc[400];
  size_t encLen = cobsEncode(plain, off, enc);

  // FIX: use delimiter framing (0x00 ... 0x00) for robustness w/ auto-dir
  RS485_SERIAL.write((uint8_t)0x00);
  RS485_SERIAL.write(enc, encLen);
  RS485_SERIAL.write((uint8_t)0x00);
  RS485_SERIAL.flush();
}

static uint8_t g_uid[6];
static uint8_t g_nodeId = 0xFF;
static bool g_streaming = false;

static Preferences g_prefs;

// Discovery (Phase A) state
static uint8_t  g_discEpoch = 0;
static bool     g_helloPending = false;
static bool     g_helloSent = false;
static uint32_t g_helloDueMs = 0;

// schedule
static uint32_t g_cycleUs = CYCLE_US;
static uint32_t g_offsetUs = 6200;
static uint8_t  g_periodCycles = 2;
static uint8_t  g_streamSof = SOF_VIB;

static uint32_t g_epochStartUs = 0;
static uint32_t g_cycleIndex = 0;

// stream seq
static uint8_t g_seqStream = 0;

static SPIClass* g_spi = nullptr;
static SPISettings adxlSPI(5000000, MSBFIRST, SPI_MODE3);

static void adxlWriteReg(uint8_t reg, uint8_t val) {
  digitalWrite(ADXL_CS_PIN, LOW);
  g_spi->beginTransaction(adxlSPI);
  g_spi->transfer(reg & 0x3F);
  g_spi->transfer(val);
  g_spi->endTransaction();
  digitalWrite(ADXL_CS_PIN, HIGH);
}

static uint8_t adxlReadReg(uint8_t reg) {
  digitalWrite(ADXL_CS_PIN, LOW);
  g_spi->beginTransaction(adxlSPI);
  g_spi->transfer(0x80 | (reg & 0x3F));
  uint8_t v = g_spi->transfer(0x00);
  g_spi->endTransaction();
  digitalWrite(ADXL_CS_PIN, HIGH);
  return v;
}

static uint8_t adxlFifoCount() {
  uint8_t n = 0;
  digitalWrite(ADXL_CS_PIN, LOW);
  g_spi->beginTransaction(adxlSPI);
  g_spi->transfer(0x39 | 0x80);
  n = g_spi->transfer(0x00);
  g_spi->endTransaction();
  digitalWrite(ADXL_CS_PIN, HIGH);
  return (uint8_t)(n & 0x3F);
}

static void adxlReadFifoSample(int16_t* x, int16_t* y, int16_t* z) {
  uint8_t b[6];
  digitalWrite(ADXL_CS_PIN, LOW);
  g_spi->beginTransaction(adxlSPI);
  g_spi->transfer(0x32 | 0x80 | 0x40);
  for (int i = 0; i < 6; i++) b[i] = g_spi->transfer(0x00);
  g_spi->endTransaction();
  digitalWrite(ADXL_CS_PIN, HIGH);

  *x = (int16_t)((uint16_t)b[0] | ((uint16_t)b[1] << 8));
  *y = (int16_t)((uint16_t)b[2] | ((uint16_t)b[3] << 8));
  *z = (int16_t)((uint16_t)b[4] | ((uint16_t)b[5] << 8));
}

static void collectVibBlockFromFifo(uint8_t* outPayload) {
  uint8_t available = adxlFifoCount();
  int16_t x=0, y=0, z=0;
  uint8_t* p = outPayload;

  uint8_t toRead = (available >= VIB_SAMPLES_PER_BLOCK) ? VIB_SAMPLES_PER_BLOCK : available;

  for (uint8_t i = 0; i < toRead; i++) {
    adxlReadFifoSample(&x, &y, &z);
    *p++ = (uint8_t)(x & 0xFF); *p++ = (uint8_t)(x >> 8);
    *p++ = (uint8_t)(y & 0xFF); *p++ = (uint8_t)(y >> 8);
    *p++ = (uint8_t)(z & 0xFF); *p++ = (uint8_t)(z >> 8);
  }

  for (uint8_t i = toRead; i < VIB_SAMPLES_PER_BLOCK; i++) {
    *p++ = (uint8_t)(x & 0xFF); *p++ = (uint8_t)(x >> 8);
    *p++ = (uint8_t)(y & 0xFF); *p++ = (uint8_t)(y >> 8);
    *p++ = (uint8_t)(z & 0xFF); *p++ = (uint8_t)(z >> 8);
  }
}

static uint8_t odrToBwRate() {
  if (ADXL_ODR_HZ >= 3200) return 0x0F;
  if (ADXL_ODR_HZ >= 1600) return 0x0E;
  return 0x0D;
}

static void adxlInit() {
  pinMode(ADXL_CS_PIN, OUTPUT);
  digitalWrite(ADXL_CS_PIN, HIGH);

  g_spi = new SPIClass(VSPI);
  g_spi->begin(ADXL_SCK_PIN, ADXL_MISO_PIN, ADXL_MOSI_PIN, ADXL_CS_PIN);
  delay(20);

  uint8_t devid = adxlReadReg(0x00);
  DBG_SERIAL.printf("[ADXL] DEVID=0x%02X\n", devid);

  if (devid != 0xE5) {
    DBG_SERIAL.println("[ADXL] ERROR: SPI not working (check wiring/pins/module).");
    return;
  }

  adxlWriteReg(0x2D, 0x08);
  adxlWriteReg(0x31, 0x0B);
  adxlWriteReg(0x2C, odrToBwRate());

  adxlWriteReg(0x38, 0x00);
  delay(2);
  adxlWriteReg(0x38, 0x80 | 0x1F);
}

static void writePreamble() {
  for (int i = 0; i < STREAM_PREAMBLE_LEN; i++) RS485_SERIAL.write((uint8_t)0x55);
}

static void sendVibFrame(const uint8_t* payload, size_t payloadLen) {
  writePreamble();
  uint8_t seq = g_seqStream++;
  RS485_SERIAL.write((uint8_t)SOF_VIB);
  RS485_SERIAL.write((uint8_t)seq);
  RS485_SERIAL.write(payload, payloadLen);

  uint16_t crc = 0xFFFF;
  uint8_t hdr[2] = { SOF_VIB, seq };
  crc = crc16_update(crc, hdr, sizeof(hdr));
  crc = crc16_update(crc, payload, payloadLen);

  RS485_SERIAL.write((uint8_t)(crc & 0xFF));
  RS485_SERIAL.write((uint8_t)(crc >> 8));
  RS485_SERIAL.flush();
}

static inline void rs_rx_enable(bool en) {
  if (en) {
    uart_enable_rx_intr(RS_UART);
  } else {
    uart_disable_rx_intr(RS_UART);
    uart_flush_input(RS_UART); // drop any accumulated bytes (fast)
  }
}

static bool tryParseResyncRaw_budget(uint32_t budget_us) {
  static uint8_t streak = 0;
  static uint8_t state = 0;
  static uint8_t buf[8];
  static uint8_t idx = 0;

  uint32_t t0 = micros();
  bool got = false;

  while ((uint32_t)(micros() - t0) < budget_us) {
    int avail = RS485_SERIAL.available();
    if (avail <= 0) { delayMicroseconds(20); continue; }

    uint8_t b = (uint8_t)RS485_SERIAL.read();

    if (state == 0) {
      if (b == 0x55) { if (streak < 255) streak++; }
      else streak = 0;
      if (streak >= STREAM_PREAMBLE_LEN) { state = 1; streak = 0; }
    } else if (state == 1) {
      if (b == SOF_RESYNC) { state = 2; idx = 0; }
      else state = 0;
    } else if (state == 2) {
      buf[idx++] = b;
      if (idx >= sizeof(buf)) {
        uint8_t epoch = buf[0];
        uint16_t apply_us = (uint16_t)buf[1] | ((uint16_t)buf[2] << 8);
        uint16_t cyc_us   = (uint16_t)buf[3] | ((uint16_t)buf[4] << 8);
        uint16_t rx_crc   = (uint16_t)buf[5] | ((uint16_t)buf[6] << 8);

        uint8_t tmp[6];
        tmp[0] = SOF_RESYNC;
        tmp[1] = epoch;
        tmp[2] = (uint8_t)(apply_us & 0xFF);
        tmp[3] = (uint8_t)(apply_us >> 8);
        tmp[4] = (uint8_t)(cyc_us & 0xFF);
        tmp[5] = (uint8_t)(cyc_us >> 8);

        uint16_t crc = crc16_ccitt_false(tmp, sizeof(tmp));
        if (crc == rx_crc) {
          g_cycleUs = cyc_us;
          g_epochStartUs = micros() + apply_us;
          g_cycleIndex = 0;
          DBG_SERIAL.printf("[RESYNC] epoch=%u apply=%u us cycle=%u us\n", epoch, apply_us, cyc_us);
          got = true;
        }
        state = 0;
        break;
      }
    }
  }
  return got;
}

static void getUid48(uint8_t uid[6]) {
  uint64_t mac = ESP.getEfuseMac();
  uid[0] = (uint8_t)(mac >> 40);
  uid[1] = (uint8_t)(mac >> 32);
  uid[2] = (uint8_t)(mac >> 24);
  uid[3] = (uint8_t)(mac >> 16);
  uid[4] = (uint8_t)(mac >> 8);
  uid[5] = (uint8_t)(mac);
}

static void sendHelloDescr() {
  uint8_t pl[12];
  memcpy(pl, g_uid, 6);
  pl[6] = (uint8_t)DEV_VIB;
  uint16_t fs = (uint16_t)ADXL_ODR_HZ;
  pl[7] = (uint8_t)(fs & 0xFF);
  pl[8] = (uint8_t)(fs >> 8);
  pl[9]  = (uint8_t)FMT_XYZ16;
  pl[10] = 3;
  pl[11] = VIB_BLOCK_MS;

  sendCobsPacket(BROADCAST_ID, MSG_HELLO, pl, sizeof(pl));
  DBG_SERIAL.println("[MGMT] HELLO_DESCR sent");
}

static void sendIdAck() {
  uint8_t pl[7];
  memcpy(pl, g_uid, 6);
  pl[6] = g_nodeId;
  sendCobsPacket(BROADCAST_ID, MSG_ID_ACK, pl, sizeof(pl));
  DBG_SERIAL.printf("[MGMT] ID_ACK id=%u\n", g_nodeId);
}

static void handleMgmtPlain(const uint8_t* plain, size_t len) {
  if (len < sizeof(PacketHeader) + 2) return;
  const PacketHeader* h = (const PacketHeader*)plain;
  if (h->magic0 != MAGIC0 || h->magic1 != MAGIC1 || h->ver != VER) return;

  uint16_t plen = h->len;
  size_t need = sizeof(PacketHeader) + plen + 2;
  if (need != len) return;

  const uint8_t* payload = plain + sizeof(PacketHeader);
  uint16_t rx_crc = (uint16_t)plain[len - 2] | ((uint16_t)plain[len - 1] << 8);
  uint16_t crc = crc16_ccitt_false(plain, len - 2);
  if (crc != rx_crc) return;

  if (h->type == MSG_DISC_OPEN) {
    uint8_t epoch = 1;
    uint8_t rand_max = 50;
    if (plen >= 4) {
      epoch = payload[0];
      rand_max = payload[3];
      if (rand_max < 10) rand_max = 10;
      if (rand_max > 250) rand_max = 250;
    }
    if (epoch != g_discEpoch) {
      g_discEpoch = epoch;
      g_helloSent = false;
      g_helloPending = false;
    }
    if (!g_helloSent && !g_helloPending) {
      g_helloDueMs = millis() + (uint32_t)random(0, (int)rand_max + 1);
      g_helloPending = true;
    }
    return;
  }

  if (h->type == MSG_ASSIGN_ID) {
    if (plen < 7) return;
    if (memcmp(payload, g_uid, 6) != 0) return;
    g_nodeId = payload[6];
    g_prefs.begin("fast", false);
    g_prefs.putUChar("node_id", g_nodeId);
    g_prefs.end();
    DBG_SERIAL.printf("[MGMT] ASSIGNED id=%u\n", g_nodeId);
    sendIdAck();
    return;
  }

  if (h->type == MSG_SCHEDULE) {
    if (plen < 2 + 4 + 1) return;
    uint16_t start_after_ms = (uint16_t)payload[0] | ((uint16_t)payload[1] << 8);
    uint32_t cycle_us = (uint32_t)payload[2] | ((uint32_t)payload[3] << 8) | ((uint32_t)payload[4] << 16) | ((uint32_t)payload[5] << 24);
    uint8_t count = payload[6];
    const uint8_t* p = payload + 7;
    const size_t entrySize = 6 + 1 + 4 + 1 + 1;
    if ((size_t)(7 + (size_t)count * entrySize) > plen) return;

    for (uint8_t i = 0; i < count; i++) {
      const uint8_t* uid = p;
      uint32_t off_us = (uint32_t)p[7] | ((uint32_t)p[8] << 8) | ((uint32_t)p[9] << 16) | ((uint32_t)p[10] << 24);
      uint8_t period = p[11];
      uint8_t sof = p[12];

      if (memcmp(uid, g_uid, 6) == 0) {
        g_cycleUs = cycle_us;
        g_offsetUs = off_us;
        g_periodCycles = (period == 0) ? 1 : period;
        g_streamSof = sof;

        g_prefs.begin("fast", false);
        g_prefs.putUInt("cycle_us", g_cycleUs);
        g_prefs.putUInt("offset_us", g_offsetUs);
        g_prefs.putUChar("period", g_periodCycles);
        g_prefs.end();

        DBG_SERIAL.printf("[MGMT] SCHEDULE ok: start_after=%ums cycle=%lu off=%lu period=%u sof=0x%02X\n",
          start_after_ms, (unsigned long)cycle_us, (unsigned long)off_us, period, sof);

        g_epochStartUs = micros() + (uint32_t)start_after_ms * 1000UL;
        g_cycleIndex = 0;
      }
      p += entrySize;
    }
    return;
  }

  if (h->type == MSG_START_STREAM) {
    if (g_nodeId == 0xFF || g_epochStartUs == 0) {
      DBG_SERIAL.println("[MGMT] START ignored (no schedule/id)");
      return;
    }
    g_streaming = true;
    DBG_SERIAL.println("[STREAM] START");

    // STREAM: disable RX almost always (reduce load)
    rs_rx_enable(false);
    return;
  }
}

static bool     g_resyncArmed = false;
static uint32_t g_lastListenMs = 0;

void setup() {
  DBG_SERIAL.begin(DBG_BAUD);
  delay(50);

  getUid48(g_uid);
  DBG_SERIAL.printf("[BOOT] UID=%02X%02X%02X%02X%02X%02X\n", g_uid[0],g_uid[1],g_uid[2],g_uid[3],g_uid[4],g_uid[5]);

  RS485_SERIAL.begin(RS485_BAUD, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  // MGMT needs RX on
  rs_rx_enable(true);

  adxlInit();

  DBG_SERIAL.println("[BOOT] waiting MGMT_OPEN...");
}

void loop() {
  if (!g_streaming) {
    if (g_helloPending && !g_helloSent && (int32_t)(millis() - g_helloDueMs) >= 0) {
      sendHelloDescr();
      g_helloSent = true;
      g_helloPending = false;
    }
    uint8_t plain[320];
    size_t plen = 0;
    if (readCobsFrameRS(plain, sizeof(plain), &plen, 10)) {
      handleMgmtPlain(plain, plen);
    }
    delay(2);
    return;
  }

  uint32_t nowUs = micros();
  if ((int32_t)(nowUs - g_epochStartUs) < 0) {
    delay(1);
    return;
  }

  // Arm resync listen every RESYNC_EXPECT_MS (match master)
  uint32_t nowMs = millis();
// RESYNC listen: open RX at the beginning of *every* cycle (master-only window)
// This avoids "missed resync" when the CPU is busy and continuously corrects drift.
 uint32_t dtUs = (uint32_t)(nowUs - g_epochStartUs);
uint32_t phase = dtUs % g_cycleUs;


  if (phase < RESYNC_OPEN_WINDOW_US) {
    rs_rx_enable(true);

    uint32_t budget = 250; // us
    if ((uint32_t)(nowMs - g_lastListenMs) > (RESYNC_EXPECT_MS * 3UL)) budget = RESYNC_OPEN_WINDOW_US;

    if (tryParseResyncRaw_budget(budget)) {
      g_lastListenMs = nowMs;
    }

    rs_rx_enable(false);
  }

  // Compute current cycle
  uint32_t dt = nowUs - g_epochStartUs;
  uint32_t cyc = dt / g_cycleUs;
  uint32_t tIn = dt % g_cycleUs;
  g_cycleIndex = cyc;

  // Period gating
  if ((g_cycleIndex % g_periodCycles) != 0) return;

  // One send per eligible cycle
  static uint32_t lastSentCycle = 0xFFFFFFFF;
  if (lastSentCycle == g_cycleIndex) return;

  // Wait until offset
  if (tIn < g_offsetUs) return;

  // Late check (skip to avoid collisions)
  uint32_t targetAbs = g_epochStartUs + g_cycleIndex * g_cycleUs + g_offsetUs;
  if ((uint32_t)(nowUs - targetAbs) > VIB_LATE_US) {
    lastSentCycle = g_cycleIndex; // mark skipped
    return;
  }

  // OK send now
  lastSentCycle = g_cycleIndex;
  static uint8_t payload[VIB_PAYLOAD_BYTES];
  collectVibBlockFromFifo(payload);
  sendVibFrame(payload, sizeof(payload));
}
