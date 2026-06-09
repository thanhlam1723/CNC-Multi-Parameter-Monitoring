/* Temp node (MAX31865 + PT100) | SlowBus RS485
   - RS485 UART2: RX=16 TX=17
   - Master-driven: node only transmits on DISCOVER_OPEN / requests
*/

#include <Arduino.h>
#include <SPI.h>
#include <math.h>
#include <HardwareSerial.h>
#include <Preferences.h>

// ---- logging ----
#ifndef DEBUG_LOG
#define DEBUG_LOG 0
#endif
#if DEBUG_LOG
#define LOGF(...)   Serial.printf(__VA_ARGS__)
#define LOGLN(x)    Serial.println(x)
#else
#define LOGF(...)
#define LOGLN(x)
#endif


// STRICT_MASTER_DRIVEN=1: Node never transmits unless master explicitly opens discovery window
// or directly requests data. This disables announce_boot().
#define STRICT_MASTER_DRIVEN 1


#define MAX_MISO 23
#define MAX_MOSI 19
#define MAX_SCK  18
#define MAX_CS   5

#define RS485_RX 16
#define RS485_TX 17
HardwareSerial RS485(2);

// Nếu module RS485 có DE/RE:
// #define RS485_EN 4
static void rs485_set_tx(bool tx) {
  // digitalWrite(RS485_EN, tx ? HIGH : LOW);
}

static const uint8_t VER_1 = 0x01;

static const uint8_t MSG_DISCOVER_OPEN = 0x01;

static const uint8_t DISC_FLAG_FORCE_HELLO_ALL = 0x01; // master can request ALL nodes announce in the discovery window
static const uint8_t MSG_ANNOUNCE      = 0x02;
static const uint8_t MSG_ASSIGN_ADDR   = 0x03;
static const uint8_t MSG_ADDR_CONFIRM  = 0x04;

static const uint8_t MSG_DESCR_REQ     = 0x10;
static const uint8_t MSG_DESCR_RESP    = 0x11;

static const uint8_t MSG_READ_REQ      = 0x20;
static const uint8_t MSG_READ_RESP     = 0x21;

static const uint8_t ADDR_MASTER       = 0xF0;
static const uint8_t ADDR_BROADCAST    = 0xFF;

static const uint8_t DEVICE_CLASS      = 0x20; // TempNode
static const uint8_t DESCR_VER         = 1;

// kênh của node nhiệt
static const uint8_t  CH_TEMP     = 1;
static const uint16_t SCALE_TEMP  = 100; // temp_x100
static const uint8_t  UNIT_C      = 1;

static uint16_t crc16_modbus(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) crc = (crc & 1) ? ((crc >> 1) ^ 0xA001) : (crc >> 1);
  }
  return crc;
}

static size_t cobs_encode(const uint8_t* in, size_t len, uint8_t* out) {
  size_t r = 0, w = 1, code_i = 0;
  uint8_t code = 1;
  while (r < len) {
    if (in[r] == 0) { out[code_i] = code; code = 1; code_i = w++; r++; }
    else {
      out[w++] = in[r++]; code++;
      if (code == 0xFF) { out[code_i] = code; code = 1; code_i = w++; }
    }
  }
  out[code_i] = code;
  return w;
}

static size_t cobs_decode(const uint8_t* in, size_t len, uint8_t* out) {
  size_t r = 0, w = 0;
  while (r < len) {
    uint8_t code = in[r++];
    if (code == 0) return 0;
    for (uint8_t i = 1; i < code; i++) {
      if (r >= len) return 0;
      out[w++] = in[r++];
    }
    if (code != 0xFF && r < len) out[w++] = 0;
  }
  return w;
}

static size_t cobs_decode_cap(const uint8_t* in, size_t len, uint8_t* out, size_t out_cap) {
  size_t r = 0, w = 0;
  while (r < len) {
    uint8_t code = in[r++];
    if (code == 0) return 0;
    for (uint8_t i = 1; i < code; i++) {
      if (r >= len) return 0;
      if (w >= out_cap) return 0;
      out[w++] = in[r++];
    }
    if (code != 0xFF && r < len) {
      if (w >= out_cap) return 0;
      out[w++] = 0;
    }
  }
  return w;
}


static void send_frame(uint8_t msg, uint8_t flags, uint8_t dst, uint8_t src,
                       uint16_t seq, const uint8_t* payload, uint16_t payload_len) {
  uint8_t raw[256]; size_t idx = 0;
  raw[idx++] = VER_1;
  raw[idx++] = msg;
  raw[idx++] = flags;
  raw[idx++] = dst;
  raw[idx++] = src;
  raw[idx++] = (uint8_t)(seq & 0xFF);
  raw[idx++] = (uint8_t)(seq >> 8);
  raw[idx++] = (uint8_t)(payload_len & 0xFF);
  raw[idx++] = (uint8_t)(payload_len >> 8);
  if (payload_len && payload) {
    if (idx + payload_len + 2 > sizeof(raw)) return; // drop oversize
    memcpy(&raw[idx], payload, payload_len);
    idx += payload_len;
  } else {
    if (idx + 2 > sizeof(raw)) return;
  }
  uint16_t crc = crc16_modbus(raw, idx);
  raw[idx++] = (uint8_t)(crc & 0xFF);
  raw[idx++] = (uint8_t)(crc >> 8);

  uint8_t enc[300];
  size_t enc_len = cobs_encode(raw, idx, enc);
  if (enc_len == 0 || enc_len > sizeof(enc)) return;

  rs485_set_tx(true);
  RS485.write(enc, enc_len);
  RS485.write((uint8_t)0x00);
  RS485.flush();
  rs485_set_tx(false);
}

static bool recv_frame(uint8_t* out_raw, size_t out_cap, size_t* out_len) {
  static uint8_t buf[300];
  static size_t blen = 0;

  while (RS485.available()) {
    uint8_t b = (uint8_t)RS485.read();
    if (b == 0x00) {
      if (blen == 0) continue;
      uint8_t decoded[256];
      size_t dlen = cobs_decode_cap(buf, blen, decoded, sizeof(decoded));
      blen = 0;

      if (dlen < 11) return false;
      uint16_t got = (uint16_t)decoded[dlen - 2] | ((uint16_t)decoded[dlen - 1] << 8);
      uint16_t cal = crc16_modbus(decoded, dlen - 2);
      if (got != cal) return false;

      if (dlen > out_cap) return false;
      memcpy(out_raw, decoded, dlen);
      *out_len = dlen;
      return true;
    } else {
      if (blen < sizeof(buf)) buf[blen++] = b;
      else blen = 0;
    }
  }
  return false;
}

static Preferences prefs;
static uint8_t my_addr = 0x00;

// Scheduled HELLO (non-blocking) after DISCOVER_OPEN
static bool ann_pending = false;
static uint32_t ann_due_ms = 0;



static void get_uid6(uint8_t uid[6]) {
  uint64_t mac = ESP.getEfuseMac();
  uid[0] = (uint8_t)(mac >> 0);
  uid[1] = (uint8_t)(mac >> 8);
  uid[2] = (uint8_t)(mac >> 16);
  uid[3] = (uint8_t)(mac >> 24);
  uid[4] = (uint8_t)(mac >> 32);
  uid[5] = (uint8_t)(mac >> 40);
}
static bool uid_match(const uint8_t* a, const uint8_t* b) {
  for (int i = 0; i < 6; i++) if (a[i] != b[i]) return false;
  return true;
}

static uint8_t load_addr() {
  prefs.begin("slowbus", true);
  uint8_t a = prefs.getUChar("addr", 0);
  prefs.end();
  return a;
}
static void save_addr(uint8_t a) {
  prefs.begin("slowbus", false);
  prefs.putUChar("addr", a);
  prefs.end();
}
static void clear_addr() {
  prefs.begin("slowbus", false);
  prefs.remove("addr");
  prefs.end();
}

static void maybe_clear_addr_on_boot() {
  pinMode(0, INPUT_PULLUP);
  if (digitalRead(0) == HIGH) return;

  uint32_t t0 = millis();
  while (millis() - t0 < 1500) {
    if (digitalRead(0) == HIGH) return;
    delay(10);
  }
  clear_addr();
  my_addr = 0;
  LOGLN("ADDR cleared by BOOT-hold -> my_addr=0");
}

// ANNOUNCE boot vài lần (cắm/rút thấy lại nhanh)
static void announce_boot(uint16_t seq_hint = 0xBEEF, uint8_t times = 3) {
  uint8_t uid[6]; get_uid6(uid);

  uint8_t a[8];
  memcpy(a, uid, 6);
  a[6] = DEVICE_CLASS;
  a[7] = my_addr;

  for (uint8_t k = 0; k < times; k++) {
    delay((uint16_t)random(30, 180));
    send_frame(MSG_ANNOUNCE, 0x00, ADDR_MASTER, (my_addr == 0 ? 0x00 : my_addr), seq_hint + k, a, sizeof(a));
  }
  LOGF("ANNOUNCE boot x%u (my_addr=0x%02X)\n", times, my_addr);
}

// NOTE: Non-blocking sampling: start one-shot conversion, then read result later (no delay()).

static double resistance;
static double temperature;

static const double RTDa = 3.9083e-3;
static const double RTDb = -5.775e-7;

static uint16_t last_rtd_raw = 0;
static int32_t  last_temp_x100 = 0;
static uint16_t last_status = 0;
static uint32_t last_sample_ms = 0;
static bool     last_valid = false;

static const uint32_t TEMP_SAMPLE_MS = 1000;   // snapshot period
static const uint32_t MAX_CONV_MS    = 60;     // MAX31865 one-shot conversion time (typ ~55ms)

// sampler state
static bool     conv_inflight = false;
static uint32_t conv_ready_ms = 0;
static uint32_t next_sample_due_ms = 0;

static void convertToTemperature() {
  double Rt = resistance;
  Rt /= 32768.0;
  Rt *= 430.0;

  double Z1 = -RTDa;
  double Z2 = RTDa * RTDa - (4.0 * RTDb);
  double Z3 = (4.0 * RTDb) / 100.0;
  double Z4 = 2.0 * RTDb;

  double t = Z2 + (Z3 * Rt);
  t = (sqrt(t) + Z1) / Z4;

  if (t >= 0) { temperature = t; return; }

  // polynomial for negative temps
  double rpoly = Rt;
  t = -242.02;
  t += 2.2228 * rpoly;
  rpoly *= Rt; t += 2.5859e-3 * rpoly;
  rpoly *= Rt; t -= 4.8260e-6 * rpoly;
  rpoly *= Rt; t -= 2.8183e-8 * rpoly;
  rpoly *= Rt; t += 1.5243e-10 * rpoly;
  temperature = t;
}

static void max31865_start_oneshot() {
  SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE1));
  digitalWrite(MAX_CS, LOW);
  SPI.transfer(0x80);   // write config
  SPI.transfer(0xB0);   // VBIAS=1, 1-shot=1, 3-wire=1 (keep your original config)
  digitalWrite(MAX_CS, HIGH);
  SPI.endTransaction();

  conv_inflight = true;
  conv_ready_ms = millis() + MAX_CONV_MS;
}

static bool max31865_read_rtd(uint16_t* rtd_raw) {
  uint8_t reg1 = 0, reg2 = 0;

  SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE1));
  digitalWrite(MAX_CS, LOW);
  SPI.transfer(0x01);   // read RTD MSBs
  reg1 = SPI.transfer(0xFF);
  reg2 = SPI.transfer(0xFF);
  digitalWrite(MAX_CS, HIGH);
  SPI.endTransaction();

  uint16_t fullreg = ((uint16_t)reg1 << 8) | reg2;
  fullreg >>= 1; // remove fault bit
  *rtd_raw = fullreg;
  return true;
}

static void temp_sampler_step() {
  uint32_t now = millis();

  if (!conv_inflight) {
    if ((int32_t)(now - next_sample_due_ms) >= 0) {
      max31865_start_oneshot();
    }
    return;
  }

  // conversion in flight
  if ((int32_t)(now - conv_ready_ms) < 0) return;

  uint16_t rtd_raw = 0;
  bool ok = max31865_read_rtd(&rtd_raw);

  conv_inflight = false;
  next_sample_due_ms = now + TEMP_SAMPLE_MS;

  if (!ok) {
    last_status = 0x0002;
    last_valid = false;
    return;
  }

  last_rtd_raw = rtd_raw;
  resistance = (double)rtd_raw;

  convertToTemperature();

  // status bit0=ok range, bit1=bad/out-of-range
  uint16_t status = 0;
  if (temperature > -200 && temperature < 850) status |= 0x0001;
  else status |= 0x0002;

  last_temp_x100 = (int32_t)lround(temperature * 100.0);
  last_status = status;
  last_sample_ms = now;
  last_valid = true;
}
// ANNOUNCE nhanh (không delay dài) dùng cho heartbeat
static void announce_fast(uint16_t seq_hint = 0xCAFE) {
  if (my_addr == 0) return;

  uint8_t uid[6]; get_uid6(uid);

  uint8_t payload[8];
  memcpy(payload, uid, 6);
  payload[6] = DEVICE_CLASS;
  payload[7] = my_addr;
  // jitter handled by DISCOVER_OPEN scheduling; keep TX non-blocking
  send_frame(MSG_ANNOUNCE, 0x00, ADDR_MASTER, my_addr, seq_hint, payload, sizeof(payload));
}
static void announce_known(uint16_t seq_hint = 0xCAFE) {
  // ANNOUNCE lại khi master reboot (my_addr != 0)
  if (my_addr == 0) return;
  uint8_t uid[6]; get_uid6(uid);
  uint8_t payload[8];
  memcpy(payload, uid, 6);
  payload[6] = DEVICE_CLASS;
  payload[7] = my_addr;
  send_frame(MSG_ANNOUNCE, 0x00, ADDR_MASTER, my_addr, seq_hint, payload, sizeof(payload));
}


static void announce_hello(uint16_t seq_hint = 0xCAFE) {
  // Chỉ HELLO khi chưa được cấp addr
  if (my_addr != 0) return;
  uint8_t uid[6]; get_uid6(uid);
  uint8_t payload[8];
  memcpy(payload, uid, 6);
  payload[6] = DEVICE_CLASS;
  payload[7] = my_addr;
  send_frame(MSG_ANNOUNCE, 0x00, ADDR_MASTER, 0x00, seq_hint, payload, sizeof(payload));
}


void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(MAX_CS, OUTPUT);
  digitalWrite(MAX_CS, HIGH);
  SPI.begin(MAX_SCK, MAX_MISO, MAX_MOSI);

  RS485.begin(38400, SERIAL_8N1, RS485_RX, RS485_TX);

  randomSeed((uint32_t)esp_random());

  my_addr = load_addr();
  maybe_clear_addr_on_boot();

  LOGF("TEMP_NODE ready (MODE B), my_addr=0x%02X\n", my_addr);
#if !STRICT_MASTER_DRIVEN
  announce_boot();
#endif
}

void loop() {
  uint8_t raw[256];
  size_t len = 0;
  while (recv_frame(raw, sizeof(raw), &len)) {
    uint8_t ver = raw[0];
    uint8_t msg = raw[1];
    uint8_t dst = raw[3];
    uint8_t src = raw[4];
    uint16_t seq = (uint16_t)raw[5] | ((uint16_t)raw[6] << 8);
    uint16_t plen = (uint16_t)raw[7] | ((uint16_t)raw[8] << 8);
    const uint8_t* p = &raw[9];

    if (ver != VER_1) continue;

    // DISCOVERY OPEN -> schedule HELLO (non-blocking), chỉ khi chưa có addr
  if (msg == MSG_DISCOVER_OPEN && dst == ADDR_BROADCAST && src == ADDR_MASTER) {
    if (plen < 4) continue;

    uint16_t window_ms = (uint16_t)p[0] | ((uint16_t)p[1] << 8);
    uint16_t rand_max  = (uint16_t)p[2] | ((uint16_t)p[3] << 8);
    uint8_t  flags     = (plen >= 5) ? p[4] : 0;

    // Chỉ node mới (addr==0) tự HELLO; còn node đã có addr chỉ ANNOUNCE lại khi master yêu cầu FORCE_HELLO_ALL
    if (my_addr != 0 && (flags & DISC_FLAG_FORCE_HELLO_ALL) == 0) continue;

    uint16_t jitter = (rand_max == 0) ? 0 : (uint16_t)random(10, rand_max + 1);
    if (jitter > window_ms) jitter = window_ms;

    ann_due_ms = millis() + jitter;
    ann_pending = true;
    continue;
  }
  if (msg == MSG_ASSIGN_ADDR && dst == ADDR_BROADCAST && src == ADDR_MASTER && plen >= 7) {
    uint8_t uid_me[6]; get_uid6(uid_me);
    if (!uid_match(uid_me, p)) continue;

    uint8_t new_addr = p[6];
    my_addr = new_addr;
    save_addr(my_addr);

    uint8_t conf[1] = { my_addr };
    send_frame(MSG_ADDR_CONFIRM, 0x00, ADDR_MASTER, my_addr, seq, conf, 1);

    LOGF("Assigned & saved addr=0x%02X\n", my_addr);
    continue;
  }

  if (my_addr == 0x00) continue;
  if (src != ADDR_MASTER) continue;
  if (dst != my_addr) continue;

  if (msg == MSG_DESCR_REQ) {
    uint8_t uid[6]; get_uid6(uid);

    uint8_t d[14];
    int i = 0;
    for (int k = 0; k < 6; k++) d[i++] = uid[k];
    d[i++] = DEVICE_CLASS;
    d[i++] = DESCR_VER;
    d[i++] = 1;        // ch_count

    d[i++] = CH_TEMP;  // ch_id
    d[i++] = 2;        // dtype=int32
    d[i++] = (uint8_t)(SCALE_TEMP & 0xFF);
    d[i++] = (uint8_t)(SCALE_TEMP >> 8);
    d[i++] = UNIT_C;

    send_frame(MSG_DESCR_RESP, 0x00, ADDR_MASTER, my_addr, seq, d, i);
    continue;
  }

  if (msg == MSG_READ_REQ) {
    uint8_t req_count = 0;
    const uint8_t* req_list = nullptr;

    if (plen >= 1) {
      req_count = p[0];
      if (req_count > 0 && plen >= (uint16_t)(1 + req_count)) req_list = &p[1];
      else if (req_count > 0) req_count = 0;
    }

    // Use cached snapshot (non-blocking sampling)
    uint16_t status = last_valid ? last_status : 0x0002;
    int32_t temp_x100 = last_valid ? last_temp_x100 : 0;

    bool want_temp = false;
    if (plen == 0 || req_count == 0) want_temp = true;
    else {
      for (uint8_t i = 0; i < req_count; i++) {
        if (req_list[i] == CH_TEMP) { want_temp = true; break; }
      }
    }

    uint8_t payload[3 + 5];
    int w = 0;
    payload[w++] = (uint8_t)(status & 0xFF);
    payload[w++] = (uint8_t)(status >> 8);

    if (!want_temp) {
      payload[w++] = 0;
      send_frame(MSG_READ_RESP, 0x00, ADDR_MASTER, my_addr, seq, payload, w);
      continue;
    }

    payload[w++] = 1;
    payload[w++] = CH_TEMP;
    payload[w++] = (uint8_t)(temp_x100 & 0xFF);
    payload[w++] = (uint8_t)((temp_x100 >> 8) & 0xFF);
    payload[w++] = (uint8_t)((temp_x100 >> 16) & 0xFF);
    payload[w++] = (uint8_t)((temp_x100 >> 24) & 0xFF);

    send_frame(MSG_READ_RESP, 0x00, ADDR_MASTER, my_addr, seq, payload, w);
    continue;
  }
  }

  // Send scheduled HELLO without blocking
  if (ann_pending && (int32_t)(millis() - ann_due_ms) >= 0) {
    ann_pending = false;
    announce_hello();
  }

  // Background temperature sampling (non-blocking)
  temp_sampler_step();
}