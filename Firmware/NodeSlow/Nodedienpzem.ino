/* Power meter node (PZEM-004T) | SlowBus RS485
   - RS485 UART2: RX=16 TX=17
   - Master-driven: node only transmits on DISCOVER_OPEN / requests
*/

#include <Arduino.h>
#include <HardwareSerial.h>
#include <Preferences.h>
#include <PZEM004Tv30.h>
#include <math.h>

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

struct PhaseData { float v, i, p, e, f, pf; bool ok; };
#define RS485_RX 26
#define RS485_TX 27
HardwareSerial RS485(1);

// Nếu module RS485 cần DE/RE, uncomment và nối:
// #define RS485_EN 25
static void rs485_set_tx(bool tx) {
  // digitalWrite(RS485_EN, tx ? HIGH : LOW);
}

#define PZEM_RXD 16
#define PZEM_TXD 17
HardwareSerial& PZEM_UART = Serial2;

static const uint8_t PZEM_ADDR1 = 0x10;
static const uint8_t PZEM_ADDR2 = 0x11;
static const uint8_t PZEM_ADDR3 = 0x12;

PZEM004Tv30 pzem1(PZEM_UART, PZEM_RXD, PZEM_TXD, PZEM_ADDR1);
PZEM004Tv30 pzem2(PZEM_UART, PZEM_RXD, PZEM_TXD, PZEM_ADDR2);
PZEM004Tv30 pzem3(PZEM_UART, PZEM_RXD, PZEM_TXD, PZEM_ADDR3);


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

// Device class + descriptor version
static const uint8_t DEVICE_CLASS = 0x10; // PZEM node
static const uint8_t DESCR_VER    = 3;    // v3: includes name[6] per channel

// L1: 10..15, L2: 20..25, L3: 30..35
static const uint8_t CH_VOLT_L1   = 10;
static const uint8_t CH_CURR_L1   = 11;
static const uint8_t CH_POWER_L1  = 12;
static const uint8_t CH_ENERGY_L1 = 13;
static const uint8_t CH_FREQ_L1   = 14;
static const uint8_t CH_PF_L1     = 15;

static const uint8_t CH_VOLT_L2   = 20;
static const uint8_t CH_CURR_L2   = 21;
static const uint8_t CH_POWER_L2  = 22;
static const uint8_t CH_ENERGY_L2 = 23;
static const uint8_t CH_FREQ_L2   = 24;
static const uint8_t CH_PF_L2     = 25;

static const uint8_t CH_VOLT_L3   = 30;
static const uint8_t CH_CURR_L3   = 31;
static const uint8_t CH_POWER_L3  = 32;
static const uint8_t CH_ENERGY_L3 = 33;
static const uint8_t CH_FREQ_L3   = 34;
static const uint8_t CH_PF_L3     = 35;

static const uint8_t UNIT_V  = 2;
static const uint8_t UNIT_A  = 3;
static const uint8_t UNIT_W  = 4;
static const uint8_t UNIT_Wh = 5;
static const uint8_t UNIT_Hz = 6;
static const uint8_t UNIT_PF = 7;

// Data type IDs
static const uint8_t DTYPE_INT32 = 2;

// Scale: int32 / scale = value
static const uint16_t SCALE_VOLT   = 10;    // 227.9 -> 2279
static const uint16_t SCALE_CURR   = 1000;  // 0.23  -> 230
static const uint16_t SCALE_POWER  = 10;    // 41.0  -> 410
static const uint16_t SCALE_ENERGY = 100;   // 0.74  -> 74
static const uint16_t SCALE_FREQ   = 100;   // 50.00 -> 5000
static const uint16_t SCALE_PF     = 100;   // 0.77  -> 77

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

// Safe decode with output capacity limit (prevents buffer overflow on noisy bus)
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
  uint8_t raw[256];
  size_t idx = 0;

  // Guard payload to avoid buffer overflow
  const size_t HEADER_LEN = 9; // ver,msg,flags,dst,src,seq(2),plen(2)
  const size_t CRC_LEN = 2;
  if ((size_t)payload_len + HEADER_LEN + CRC_LEN > sizeof(raw)) {
    // Drop oversize frame (safer than corrupting memory)
    return;
  }

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
    memcpy(&raw[idx], payload, payload_len);
    idx += payload_len;
  }

  uint16_t crc = crc16_modbus(raw, idx);
  raw[idx++] = (uint8_t)(crc & 0xFF);
  raw[idx++] = (uint8_t)(crc >> 8);

  uint8_t enc[300];
  size_t enc_len = cobs_encode(raw, idx, enc);
  if (enc_len == 0 || enc_len > sizeof(enc)) {
    return;
  }

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

      if (dlen == 0 || dlen < 11) return false;

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

static uint8_t my_addr = 0x00; // 0 => unassigned

static Preferences prefs;

static void get_uid6(uint8_t out[6]) {
  uint64_t mac = ESP.getEfuseMac();
  for (int i = 0; i < 6; i++) out[i] = (mac >> (8 * i)) & 0xFF;
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

// Hold BOOT(GPIO0) ~1.5s at boot to clear addr
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

// Gửi ANNOUNCE vài lần để cắm/rút thấy lại nhanh hơn
static void announce_boot(uint16_t seq_hint = 0xBEEF, uint8_t times = 3) {
  uint8_t uid[6]; get_uid6(uid);

  uint8_t payload[8];
  memcpy(payload, uid, 6);
  payload[6] = DEVICE_CLASS;
  payload[7] = my_addr; // 0 nếu chưa assign

  for (uint8_t k = 0; k < times; k++) {
    delay((uint16_t)random(30, 180));
    send_frame(MSG_ANNOUNCE, 0x00, ADDR_MASTER, (my_addr == 0 ? 0x00 : my_addr), seq_hint + k, payload, sizeof(payload));
  }
  LOGF("ANNOUNCE boot x%u (my_addr=0x%02X)\n", times, my_addr);
}

// ANNOUNCE nhanh (không delay dài) dùng cho heartbeat
static void announce_fast(uint16_t seq_hint = 0xCAFE) {
  if (my_addr == 0) return;

  uint8_t uid[6]; get_uid6(uid);

  uint8_t payload[8];
  memcpy(payload, uid, 6);
  payload[6] = DEVICE_CLASS;
  payload[7] = my_addr;

  delay((uint16_t)random(0, 30));
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



static PhaseData L1, L2, L3;
static uint32_t last_sample_ms = 0;
static const uint32_t SAMPLE_MS = 1000;
static uint32_t sample_period_ms = SAMPLE_MS;        // dynamic backoff when sensor fails
static const uint32_t SAMPLE_BACKOFF_MS = 2000;      // when any PZEM read fails

// Master sends DISCOVER_OPEN; node replies (HELLO/ANNOUNCE) at a random time inside the window.
static bool ann_pending = false;
static uint32_t ann_due_ms = 0;

static bool read_one(PZEM004Tv30& dev, PhaseData& out) {
  out.v  = dev.voltage();
  out.i  = dev.current();
  out.p  = dev.power();
  out.e  = dev.energy();
  out.f  = dev.frequency();
  out.pf = dev.pf();
  out.ok = !(isnan(out.v) || isnan(out.i) || isnan(out.p) || isnan(out.f) || isnan(out.pf));
  return out.ok;
}

static void sample_all_pzems() {
  read_one(pzem1, L1); delay(25);
  read_one(pzem2, L2); delay(25);
  read_one(pzem3, L3);

  uint32_t now = millis();
  last_sample_ms = now;

  // If any phase read fails, back off sampling rate to avoid bus starvation / repeated long timeouts
  bool okAll = (L1.ok && L2.ok && L3.ok);
  sample_period_ms = okAll ? SAMPLE_MS : SAMPLE_BACKOFF_MS;
}

static inline int32_t to_i32(float x, uint16_t scale) {
  if (isnan(x) || isinf(x)) return 0;
  return (int32_t)lroundf(x * (float)scale);
}

static bool get_ch_value_i32(uint8_t ch_id, int32_t* out_val) {
  // L1
  if (ch_id == CH_VOLT_L1)   { *out_val = to_i32(L1.v,  SCALE_VOLT);   return true; }
  if (ch_id == CH_CURR_L1)   { *out_val = to_i32(L1.i,  SCALE_CURR);   return true; }
  if (ch_id == CH_POWER_L1)  { *out_val = to_i32(L1.p,  SCALE_POWER);  return true; }
  if (ch_id == CH_ENERGY_L1) { *out_val = to_i32(L1.e,  SCALE_ENERGY); return true; }
  if (ch_id == CH_FREQ_L1)   { *out_val = to_i32(L1.f,  SCALE_FREQ);   return true; }
  if (ch_id == CH_PF_L1)     { *out_val = to_i32(L1.pf, SCALE_PF);     return true; }
  // L2
  if (ch_id == CH_VOLT_L2)   { *out_val = to_i32(L2.v,  SCALE_VOLT);   return true; }
  if (ch_id == CH_CURR_L2)   { *out_val = to_i32(L2.i,  SCALE_CURR);   return true; }
  if (ch_id == CH_POWER_L2)  { *out_val = to_i32(L2.p,  SCALE_POWER);  return true; }
  if (ch_id == CH_ENERGY_L2) { *out_val = to_i32(L2.e,  SCALE_ENERGY); return true; }
  if (ch_id == CH_FREQ_L2)   { *out_val = to_i32(L2.f,  SCALE_FREQ);   return true; }
  if (ch_id == CH_PF_L2)     { *out_val = to_i32(L2.pf, SCALE_PF);     return true; }
  // L3
  if (ch_id == CH_VOLT_L3)   { *out_val = to_i32(L3.v,  SCALE_VOLT);   return true; }
  if (ch_id == CH_CURR_L3)   { *out_val = to_i32(L3.i,  SCALE_CURR);   return true; }
  if (ch_id == CH_POWER_L3)  { *out_val = to_i32(L3.p,  SCALE_POWER);  return true; }
  if (ch_id == CH_ENERGY_L3) { *out_val = to_i32(L3.e,  SCALE_ENERGY); return true; }
  if (ch_id == CH_FREQ_L3)   { *out_val = to_i32(L3.f,  SCALE_FREQ);   return true; }
  if (ch_id == CH_PF_L3)     { *out_val = to_i32(L3.pf, SCALE_PF);     return true; }
  return false;
}

void setup() {
  Serial.begin(115200);
  delay(200);
  randomSeed((uint32_t)ESP.getEfuseMac());

  RS485.begin(38400, SERIAL_8N1, RS485_RX, RS485_TX);

  PZEM_UART.begin(9600, SERIAL_8N1, PZEM_RXD, PZEM_TXD);
  PZEM_UART.setTimeout(80); // reduce worst-case blocking inside library reads
  delay(200);

  my_addr = load_addr();
  maybe_clear_addr_on_boot();

  LOGLN("PZEM_NODE_3PH ready (MODE B)");
  LOGF("PZEM addrs: 0x%02X 0x%02X 0x%02X\n",
                pzem1.readAddress(), pzem2.readAddress(), pzem3.readAddress());
  LOGF("Boot my_addr=0x%02X (RS485 UART1 RX=%d TX=%d)\n", my_addr, RS485_RX, RS485_TX);
  // Avoid heavy sensor I/O before address assignment (keeps discovery responsive)
  if (my_addr != 0x00) {
    sample_all_pzems();
  } else {
    last_sample_ms = millis();
  }
#if !STRICT_MASTER_DRIVEN
  announce_boot();
#endif
}

void loop() {
  uint8_t raw[256];
  size_t len = 0;
  while (recv_frame(raw, sizeof(raw), &len)) {
    uint8_t ver = raw[0];
    if (ver != VER_1) continue;
  
    uint8_t msg  = raw[1];
    uint8_t dst  = raw[3];
    uint8_t src  = raw[4];
    uint16_t seq = (uint16_t)raw[5] | ((uint16_t)raw[6] << 8);
    uint16_t plen = (uint16_t)raw[7] | ((uint16_t)raw[8] << 8);
    const uint8_t* p = &raw[9];
  
    // DISCOVERY OPEN -> schedule HELLO (non-blocking), chỉ khi chưa có addr
    // DISCOVERY OPEN -> schedule ANNOUNCE (non-blocking)
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
  // ASSIGN ADDR (broadcast + uid match) -> đổi addr + SAVE
  if (msg == MSG_ASSIGN_ADDR && dst == ADDR_BROADCAST && src == ADDR_MASTER) {
    if (plen < 7) continue;
    uint8_t uid_me[6]; get_uid6(uid_me);
    if (memcmp(p, uid_me, 6) != 0) continue;
  
    uint8_t new_addr = p[6];
    my_addr = new_addr;
    save_addr(my_addr);
  
    uint8_t conf[1] = { my_addr };
    send_frame(MSG_ADDR_CONFIRM, 0x00, ADDR_MASTER, my_addr, seq, conf, sizeof(conf));
    LOGF("Assigned & saved addr=0x%02X\n", my_addr);
    continue;
  }
  
  // các msg sau phải gửi đúng addr node
  if (my_addr == 0) continue;
  if (src != ADDR_MASTER) continue;
  if (dst != my_addr) continue;
  
  // DESCRIPTOR
  if (msg == MSG_DESCR_REQ) {
    uint8_t uid[6]; get_uid6(uid);
  
    const uint8_t ch_count = 18;
    uint8_t d[6 + 1 + 1 + 1 + ch_count * 11];
    int i = 0;
  
    memcpy(&d[i], uid, 6); i += 6;
    d[i++] = DEVICE_CLASS;
    d[i++] = DESCR_VER;
    d[i++] = ch_count;
  
    auto add_ch = [&](uint8_t ch_id, uint8_t kind, uint8_t unit, uint16_t scale, const char* name6) {
      d[i++] = ch_id;
      d[i++] = kind; // ver>=3: kind (can be 0 if unused)
      d[i++] = unit;
      d[i++] = (uint8_t)(scale & 0xFF);
      d[i++] = (uint8_t)(scale >> 8);
      memcpy(&d[i], name6, 6); i += 6;
    };
  
    // L1
    add_ch(CH_VOLT_L1,   0x00, UNIT_V,  SCALE_VOLT,   "V_L1 ");
    add_ch(CH_CURR_L1,   0x00, UNIT_A,  SCALE_CURR,   "A_L1 ");
    add_ch(CH_POWER_L1,  0x00, UNIT_W,  SCALE_POWER,  "P_L1 ");
    add_ch(CH_ENERGY_L1, 0x00, UNIT_Wh, SCALE_ENERGY, "E_L1 ");
    add_ch(CH_FREQ_L1,   0x00, UNIT_Hz, SCALE_FREQ,   "F_L1 ");
    add_ch(CH_PF_L1,     0x00, UNIT_PF, SCALE_PF,     "PF_L1");
    // L2
    add_ch(CH_VOLT_L2,   0x00, UNIT_V,  SCALE_VOLT,   "V_L2 ");
    add_ch(CH_CURR_L2,   0x00, UNIT_A,  SCALE_CURR,   "A_L2 ");
    add_ch(CH_POWER_L2,  0x00, UNIT_W,  SCALE_POWER,  "P_L2 ");
    add_ch(CH_ENERGY_L2, 0x00, UNIT_Wh, SCALE_ENERGY, "E_L2 ");
    add_ch(CH_FREQ_L2,   0x00, UNIT_Hz, SCALE_FREQ,   "F_L2 ");
    add_ch(CH_PF_L2,     0x00, UNIT_PF, SCALE_PF,     "PF_L2");
    // L3
    add_ch(CH_VOLT_L3,   0x00, UNIT_V,  SCALE_VOLT,   "V_L3 ");
    add_ch(CH_CURR_L3,   0x00, UNIT_A,  SCALE_CURR,   "A_L3 ");
    add_ch(CH_POWER_L3,  0x00, UNIT_W,  SCALE_POWER,  "P_L3 ");
    add_ch(CH_ENERGY_L3, 0x00, UNIT_Wh, SCALE_ENERGY, "E_L3 ");
    add_ch(CH_FREQ_L3,   0x00, UNIT_Hz, SCALE_FREQ,   "F_L3 ");
    add_ch(CH_PF_L3,     0x00, UNIT_PF, SCALE_PF,     "PF_L3");
  
    send_frame(MSG_DESCR_RESP, 0x00, ADDR_MASTER, my_addr, seq, d, (uint16_t)i);
    continue;
  }
  
  // READ
  if (msg == MSG_READ_REQ) {
    if (plen < 1) continue;
  
    uint8_t req_count = p[0];
    const uint8_t* req_list = &p[1];
  
    uint16_t status = 0;
    bool ok1 = L1.ok, ok2 = L2.ok, ok3 = L3.ok;
    if (ok1 && ok2 && ok3) status |= 0x0001; else status |= 0x0002;
    if (!ok1) status |= (1u << 2);
    if (!ok2) status |= (1u << 3);
    if (!ok3) status |= (1u << 4);
    if (millis() - last_sample_ms > 3000) status |= (1u << 5);
  
    uint8_t out[220];
    int w = 0;
    out[w++] = (uint8_t)(status & 0xFF);
    out[w++] = (uint8_t)(status >> 8);
  
    uint8_t send_count = 0;
    int count_pos = w++;
  
    auto push_pair = [&](uint8_t ch_id) {
      int32_t v;
      if (!get_ch_value_i32(ch_id, &v)) return; // no 'continue' inside lambda
      out[w++] = ch_id;
      out[w++] = (uint8_t)(v & 0xFF);
      out[w++] = (uint8_t)((v >> 8) & 0xFF);
      out[w++] = (uint8_t)((v >> 16) & 0xFF);
      out[w++] = (uint8_t)((v >> 24) & 0xFF);
      send_count++;
    };
  
    if (req_count == 0) {
      push_pair(CH_VOLT_L1);   push_pair(CH_CURR_L1);   push_pair(CH_POWER_L1);
      push_pair(CH_ENERGY_L1); push_pair(CH_FREQ_L1);   push_pair(CH_PF_L1);
  
      push_pair(CH_VOLT_L2);   push_pair(CH_CURR_L2);   push_pair(CH_POWER_L2);
      push_pair(CH_ENERGY_L2); push_pair(CH_FREQ_L2);   push_pair(CH_PF_L2);
  
      push_pair(CH_VOLT_L3);   push_pair(CH_CURR_L3);   push_pair(CH_POWER_L3);
      push_pair(CH_ENERGY_L3); push_pair(CH_FREQ_L3);   push_pair(CH_PF_L3);
    } else {
      uint8_t n = req_count;
      if (n > 40) n = 40;
      for (uint8_t k = 0; k < n; k++) {
        if (w > (int)sizeof(out) - 10) break;
        push_pair(req_list[k]);
      }
    }
  
    out[count_pos] = send_count;
    send_frame(MSG_READ_RESP, 0x00, ADDR_MASTER, my_addr, seq, out, (uint16_t)w);
    continue;
  }
  }

// Send scheduled HELLO/ANNOUNCE (only after master opens discovery window)
if (ann_pending && (int32_t)(millis() - ann_due_ms) >= 0) {
  ann_pending = false;
  if (my_addr == 0) announce_hello(); else announce_known();
}

// Background sampling (cache only). Do not sample before address assignment to keep bus responsive.
if (my_addr != 0x00 && (uint32_t)(millis() - last_sample_ms) >= sample_period_ms) {
  sample_all_pzems();
}
}

