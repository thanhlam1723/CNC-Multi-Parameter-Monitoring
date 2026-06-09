#include "slowbus.h"
#include "usbmux.h"

namespace slowbus {
  Stream& DATA = ::Serial; // USB CDC
  Stream& DBG  = ::DBG;    // UART0
}

static void assert_gpio_ok(int pin, const char* name){
  if (pin < 0 || pin > 48 || !GPIO_IS_VALID_GPIO((gpio_num_t)pin)) {
    DBG.printf("INVALID GPIO %s=%d\n", name, pin);
    while(1) delay(1000);
  }
}
static void assert_gpio_output_ok(int pin, const char* name){
  assert_gpio_ok(pin, name);
  if (!GPIO_IS_VALID_OUTPUT_GPIO((gpio_num_t)pin)) {
    DBG.printf("NOT OUTPUT GPIO %s=%d\n", name, pin);
    while(1) delay(1000);
  }
}

namespace slowbus {

enum NodeState : uint8_t {
  ST_UNASSIGNED = 0,
  ST_ONLINE,
  ST_SUSPECT,
  ST_OFFLINE
};

static inline const char* state_to_str(NodeState s) {
  switch (s) {
    case ST_UNASSIGNED: return "UNASSIGNED";
    case ST_ONLINE:     return "ONLINE";
    case ST_SUSPECT:    return "SUSPECT";
    case ST_OFFLINE:    return "OFFLINE";
    default:            return "?";
  }
}

namespace logcfg {
  static bool SLOW_PRINT_DATA     = false;
  static bool LOG_ERR_SLOW_DETAIL = false;
  static bool LOG_STATE_CHANGES   = true;
}

static uint32_t slow_rx_ok       = 0;
static uint32_t slow_err_crc     = 0;
static uint32_t slow_err_cobs    = 0;
static uint32_t slow_err_short   = 0;
static uint32_t slow_err_toolong = 0;
static uint32_t slow_err_overflow= 0;

static uint32_t slow_poll_timeout = 0;
static uint32_t slow_join_seen    = 0;
static uint32_t slow_join_ok      = 0;
static uint32_t slow_last_err_log_ms = 0;

static uint32_t g_node_state_changes = 0;

#define RS485_RX 7
#define RS485_TX 6
static const uint32_t SLOW_BAUD = 38400;

#define RS485 Serial1

static void rs485_set_tx(bool tx) { (void)tx; }

static const uint8_t VER_1 = 0x01;
static const uint8_t DISC_FLAG_FORCE_HELLO_ALL = 0x01;

static const uint8_t MSG_DISCOVER_OPEN = 0x01;
static const uint8_t MSG_ANNOUNCE      = 0x02;
static const uint8_t MSG_ASSIGN_ADDR   = 0x03;
static const uint8_t MSG_ADDR_CONFIRM  = 0x04;

static const uint8_t MSG_DESCR_REQ     = 0x10;
static const uint8_t MSG_DESCR_RESP    = 0x11;
static const uint8_t MSG_READ_REQ      = 0x20;
static const uint8_t MSG_READ_RESP     = 0x21;

static const uint8_t ADDR_MASTER       = 0xF0;
static const uint8_t ADDR_BROADCAST    = 0xFF;

static const uint8_t UNIT_C  = 1;
static const uint8_t UNIT_V  = 2;
static const uint8_t UNIT_A  = 3;
static const uint8_t UNIT_W  = 4;
static const uint8_t UNIT_Wh = 5;
static const uint8_t UNIT_Hz = 6;
static const uint8_t UNIT_PF = 7;

static const char* unit_str(uint8_t u) {
  switch (u) {
    case UNIT_C:  return "C";
    case UNIT_V:  return "V";
    case UNIT_A:  return "A";
    case UNIT_W:  return "W";
    case UNIT_Wh: return "Wh";
    case UNIT_Hz: return "Hz";
    case UNIT_PF: return "PF";
    default: return "U?";
  }
}

static const char* ch_name(uint8_t ch) {
  switch (ch) {
    case 1:  return "TEMP";
    case 10: return "VOLT_L1"; case 11: return "CURR_L1"; case 12: return "POWER_L1";
    case 13: return "ENERGY_L1"; case 14: return "FREQ_L1"; case 15: return "PF_L1";
    case 20: return "VOLT_L2"; case 21: return "CURR_L2"; case 22: return "POWER_L2";
    case 23: return "ENERGY_L2"; case 24: return "FREQ_L2"; case 25: return "PF_L2";
    case 30: return "VOLT_L3"; case 31: return "CURR_L3"; case 32: return "POWER_L3";
    case 33: return "ENERGY_L3"; case 34: return "FREQ_L3"; case 35: return "PF_L3";
    default: return "CH?";
  }
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

static uint16_t crc16_modbus(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) crc = (crc & 1) ? ((crc >> 1) ^ 0xA001) : (crc >> 1);
  }
  return crc;
}

static const uint8_t SLOW_SCHEMA_VER = 1;
static const uint8_t USBMSG_SLOW_SNAPSHOT = 1; 

#if ARDUINO_USB_CDC_ON_BOOT
#else

#endif

static uint32_t usb_seq = 1;

static void usb_write_frame_cobs_crc(const uint8_t* payload, size_t payload_len) {

  usbmux::slow_snapshot(payload, (uint16_t)payload_len);
}

static void usb_emit_slow_snapshot(const uint8_t uid[6],
                                  uint8_t addr,
                                  uint16_t status,
                                  const uint8_t* ch_ids,
                                  const int32_t* values_raw,
                                  const uint16_t* scales,
                                  const uint8_t* units,
                                  uint8_t count) {

  if (count > 60) count = 60;

  uint8_t payload[520];
  size_t i = 0;

  payload[i++] = SLOW_SCHEMA_VER;
  payload[i++] = USBMSG_SLOW_SNAPSHOT;

  uint32_t seq = usb_seq++;
  payload[i++] = (uint8_t)(seq & 0xFF);
  payload[i++] = (uint8_t)((seq >> 8) & 0xFF);
  payload[i++] = (uint8_t)((seq >> 16) & 0xFF);
  payload[i++] = (uint8_t)((seq >> 24) & 0xFF);

  uint32_t ts = millis();
  payload[i++] = (uint8_t)(ts & 0xFF);
  payload[i++] = (uint8_t)((ts >> 8) & 0xFF);
  payload[i++] = (uint8_t)((ts >> 16) & 0xFF);
  payload[i++] = (uint8_t)((ts >> 24) & 0xFF);

  memcpy(&payload[i], uid, 6); i += 6;
  payload[i++] = addr;

  payload[i++] = (uint8_t)(status & 0xFF);
  payload[i++] = (uint8_t)(status >> 8);

  payload[i++] = count;

  for (uint8_t k = 0; k < count; k++) {
    uint8_t ch = ch_ids[k];
    int32_t v  = values_raw[k];
    uint16_t sc = scales[k];
    uint8_t  un = units[k];

    payload[i++] = ch;

    payload[i++] = (uint8_t)(v & 0xFF);
    payload[i++] = (uint8_t)((v >> 8) & 0xFF);
    payload[i++] = (uint8_t)((v >> 16) & 0xFF);
    payload[i++] = (uint8_t)((v >> 24) & 0xFF);

    payload[i++] = (uint8_t)(sc & 0xFF);
    payload[i++] = (uint8_t)(sc >> 8);

    payload[i++] = un;
  }

  usb_write_frame_cobs_crc(payload, i);
}

static void uid_to_str(const uint8_t uid[6], char out[13]) {
  snprintf(out, 13, "%02X%02X%02X%02X%02X%02X",
           uid[5], uid[4], uid[3], uid[2], uid[1], uid[0]);
}

static const char* NVS_REG_NS   = "registry";
static const char* NVS_REG_CNT  = "count";
static const char* NVS_REG_UIDS = "uids";
static const uint8_t MAX_KNOWN_UIDS = 32;

static bool reg_load_uids(uint8_t uids[][6], uint8_t &count) {
  Preferences p;
  if (!p.begin(NVS_REG_NS, true)) { count = 0; return false; }
  count = p.getUChar(NVS_REG_CNT, 0);
  if (count > MAX_KNOWN_UIDS) count = MAX_KNOWN_UIDS;

  size_t need = (size_t)count * 6;
  if (need > 0) {
    size_t got = p.getBytes(NVS_REG_UIDS, uids, need);
    if (got != need) count = 0;
  }
  p.end();
  return true;
}

static void reg_add_uid(const uint8_t uid[6]) {
  uint8_t uids[MAX_KNOWN_UIDS][6];
  uint8_t count = 0;
  reg_load_uids(uids, count);

  for (uint8_t i = 0; i < count; i++) {
    if (memcmp(uids[i], uid, 6) == 0) return;
  }
  if (count >= MAX_KNOWN_UIDS) return;

  memcpy(uids[count], uid, 6);
  count++;

  Preferences p;
  if (!p.begin(NVS_REG_NS, false)) return;
  p.putUChar(NVS_REG_CNT, count);
  p.putBytes(NVS_REG_UIDS, uids, (size_t)count * 6);
  p.end();
}

static uint8_t slow_nvs_get_addr(const uint8_t uid[6]) {
  Preferences p;
  p.begin("slowmap", true);
  char k[13]; uid_to_str(uid, k);
  uint8_t a = p.getUChar(k, 0);
  p.end();
  return a;
}
static void slow_nvs_set_addr(const uint8_t uid[6], uint8_t addr) {
  Preferences p;
  p.begin("slowmap", false);
  char k[13]; uid_to_str(uid, k);
  p.putUChar(k, addr);
  p.end();
  reg_add_uid(uid);
}

struct NodeInfo {
  uint8_t addr = 0;
  uint8_t uid[6] = {0};
  uint8_t device_class = 0;
  uint8_t descr_ver = 0;
  uint8_t ch_count = 0;

  struct Ch {
    uint8_t ch_id = 0;
    uint8_t dtype = 0;     
    uint16_t scale = 1;
    uint8_t unit = 0;
  } ch[64];

  bool known = false;
  bool managed = false;
  NodeState state = ST_UNASSIGNED;
  uint8_t fail_slow = 0;

  bool online = false;
  uint32_t last_seen_ms = 0;
};

static NodeInfo nodes[32];
static NodeState last_logged_state[32];
static int node_count = 0;

static int find_node_by_uid(const uint8_t uid[6]) {
  for (int i = 0; i < node_count; i++) if (memcmp(nodes[i].uid, uid, 6) == 0) return i;
  return -1;
}
static int find_node_by_addr(uint8_t addr) {
  for (int i = 0; i < node_count; i++) if (nodes[i].addr == addr) return i;
  return -1;
}

static uint16_t find_scale(const NodeInfo& n, uint8_t ch_id) {
  for (int i = 0; i < n.ch_count; i++) if (n.ch[i].ch_id == ch_id) return n.ch[i].scale ? n.ch[i].scale : 1;
  return 1;
}
static uint8_t find_unit(const NodeInfo& n, uint8_t ch_id) {
  for (int i = 0; i < n.ch_count; i++) if (n.ch[i].ch_id == ch_id) return n.ch[i].unit;
  return 0;
}

static const uint8_t USBMSG_SLOW_DESCR = 2;

static void usb_emit_slow_descr(const NodeInfo& n) {

  uint8_t payload[520];
  size_t i = 0;

  payload[i++] = SLOW_SCHEMA_VER;
  payload[i++] = USBMSG_SLOW_DESCR;

  uint32_t seq = usb_seq++;
  payload[i++] = (uint8_t)(seq & 0xFF);
  payload[i++] = (uint8_t)((seq >> 8) & 0xFF);
  payload[i++] = (uint8_t)((seq >> 16) & 0xFF);
  payload[i++] = (uint8_t)((seq >> 24) & 0xFF);

  uint32_t ts = millis();
  payload[i++] = (uint8_t)(ts & 0xFF);
  payload[i++] = (uint8_t)((ts >> 8) & 0xFF);
  payload[i++] = (uint8_t)((ts >> 16) & 0xFF);
  payload[i++] = (uint8_t)((ts >> 24) & 0xFF);

  memcpy(&payload[i], n.uid, 6); i += 6;
  payload[i++] = n.addr;
  payload[i++] = n.device_class;
  payload[i++] = n.descr_ver;
  uint8_t count = (n.ch_count > 64) ? 64 : n.ch_count;
  payload[i++] = count;

  for (uint8_t k = 0; k < count; k++) {
    payload[i++] = n.ch[k].ch_id;
    payload[i++] = n.ch[k].dtype;
    uint16_t sc = n.ch[k].scale ? n.ch[k].scale : 1;
    payload[i++] = (uint8_t)(sc & 0xFF);
    payload[i++] = (uint8_t)(sc >> 8);
    payload[i++] = n.ch[k].unit;
    if (i + 5 >= sizeof(payload)) break;
  }

  usb_write_frame_cobs_crc(payload, i);
}

static uint16_t g_seq = 1;
static uint8_t next_addr = 1;

static uint8_t alloc_addr() {
  for (;;) {
    uint8_t a = next_addr++;
    if (a == 0 || a == ADDR_MASTER || a == ADDR_BROADCAST) continue;
    if (find_node_by_addr(a) < 0) return a;
    if (next_addr >= 200) next_addr = 1;
  }
}
static bool slow_addr_in_use_online(uint8_t addr, int except_idx = -1) {
  if (addr == 0 || addr == ADDR_MASTER || addr == ADDR_BROADCAST) return true;
  for (int i = 0; i < node_count; i++) {
    if (i == except_idx) continue;
    if (!nodes[i].online) continue;
    if (nodes[i].addr == addr) return true;
  }
  return false;
}
static uint8_t alloc_addr_online(int except_idx = -1) {
  for (uint16_t a = 1; a < 200; a++) {
    if (!slow_addr_in_use_online((uint8_t)a, except_idx)) return (uint8_t)a;
  }
  return 0;
}

static void send_frame(uint8_t msg, uint8_t flags, uint8_t dst, uint8_t src,
                       uint16_t seq, const uint8_t* payload, uint16_t payload_len) {
  uint8_t raw[256];
  size_t idx = 0;

  const uint16_t MAX_PAYLOAD = (uint16_t)(sizeof(raw) - 9 - 2);
  if (payload_len > MAX_PAYLOAD) {

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
  if (enc_len == 0 || enc_len > sizeof(enc)) return;

  rs485_set_tx(true);

  delayMicroseconds(300);        

  RS485.write(enc, enc_len);
  RS485.write((uint8_t)0x00);
  RS485.flush();

  delayMicroseconds(2000);       
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

      if (dlen == 0) { slow_err_cobs++; if (logcfg::LOG_ERR_SLOW_DETAIL && (int32_t)(millis()-slow_last_err_log_ms) >= 1000) { slow_last_err_log_ms = millis(); DBG.println("[SLOW] COBS decode fail"); } continue; }
      if (dlen < 11) { slow_err_short++; if (logcfg::LOG_ERR_SLOW_DETAIL && (int32_t)(millis()-slow_last_err_log_ms) >= 1000) { slow_last_err_log_ms = millis(); DBG.printf("[SLOW] short frame dlen=%u\n", (unsigned)dlen); } continue; }

      uint16_t got = (uint16_t)decoded[dlen - 2] | ((uint16_t)decoded[dlen - 1] << 8);
      uint16_t cal = crc16_modbus(decoded, dlen - 2);
      if (got != cal) { slow_err_crc++; if (logcfg::LOG_ERR_SLOW_DETAIL && (int32_t)(millis()-slow_last_err_log_ms) >= 1000) { slow_last_err_log_ms = millis(); DBG.println("[SLOW] CRC fail"); } continue; }

      if (dlen > out_cap) { slow_err_toolong++; if (logcfg::LOG_ERR_SLOW_DETAIL && (int32_t)(millis()-slow_last_err_log_ms) >= 1000) { slow_last_err_log_ms = millis(); DBG.printf("[SLOW] toolong dlen=%u cap=%u\n", (unsigned)dlen, (unsigned)out_cap); } continue; }

      memcpy(out_raw, decoded, dlen);
      *out_len = dlen;
      slow_rx_ok++;
      return true;
    } else {
      if (blen < sizeof(buf)) buf[blen++] = b;
      else { blen = 0; slow_err_overflow++; if (logcfg::LOG_ERR_SLOW_DETAIL && (int32_t)(millis()-slow_last_err_log_ms) >= 1000) { slow_last_err_log_ms = millis(); DBG.println("[SLOW] rx overflow"); } }
    }
  }
  return false;
}

static void open_discovery_window(uint16_t window_ms = 1200, uint16_t rand_max = 250, uint8_t flags = 0) {
  uint8_t p[5];
  p[0] = (uint8_t)(window_ms & 0xFF);
  p[1] = (uint8_t)(window_ms >> 8);
  p[2] = (uint8_t)(rand_max & 0xFF);
  p[3] = (uint8_t)(rand_max >> 8);
  p[4] = flags;
  send_frame(MSG_DISCOVER_OPEN, 0x00, ADDR_BROADCAST, ADDR_MASTER, g_seq++, p, sizeof(p));
}

static bool     mgmt_active   = false;
static uint32_t mgmt_until_ms = 0;
static bool     mgmt_request  = false;

static bool     need_assign[32] = {0};
static bool     need_assign_specific[32] = {0};
static uint8_t  need_assign_addr[32] = {0};

enum TxType : uint8_t { TX_NONE=0, TX_DESCR=1, TX_READ=2, TX_ASSIGN=3 };

struct TxState {
  TxType type = TX_NONE;
  uint8_t addr = 0;
  uint16_t seq = 0;
  uint32_t deadline_ms = 0;
  uint8_t node_index = 0;
  uint8_t retry = 0;
};

static TxState tx;

static const uint32_t OFFLINE_AFTER_MS     = 10000;
static const uint32_t DISCOVERY_PERIOD_MS  = 25000;

static const uint16_t DISC_WINDOW_MS = 350;
static const uint16_t DISC_RAND_MS   = 250;
static const uint16_t DISC_GUARD_MS  = 50;

static uint32_t last_disc_ms = 0;
static uint32_t last_poll_ms = 0;
static uint8_t  poll_idx = 0;

static bool assign_addr_to_uid(const uint8_t uid[6], int idx, const char* uid_s);
static bool assign_addr_to_uid_specific(const uint8_t uid[6], int idx, const char* uid_s, uint8_t new_addr);

static bool handle_announce_runtime(const uint8_t* raw, size_t len) {
  if (len < 11) return false;
  if (raw[1] != MSG_ANNOUNCE) return false;
  if (raw[3] != ADDR_MASTER) return false;

  slow_join_seen++;
  uint16_t plen = (uint16_t)raw[7] | ((uint16_t)raw[8] << 8);
  if (plen < 8) return false;

  const uint8_t* p = &raw[9];
  uint8_t uid[6]; memcpy(uid, p, 6); p += 6;
  uint8_t dev_class = *p++;
  uint8_t cur_addr  = *p++;

  char uid_s[13]; uid_to_str(uid, uid_s);

  int idx_uid  = find_node_by_uid(uid);
  if (idx_uid < 0) {
    if (node_count >= (int)(sizeof(nodes) / sizeof(nodes[0]))) return true;
    idx_uid = node_count++;
    memset(&nodes[idx_uid], 0, sizeof(NodeInfo));
    memcpy(nodes[idx_uid].uid, uid, 6);
    last_logged_state[idx_uid] = ST_UNASSIGNED;
  }

  NodeInfo& n = nodes[idx_uid];
  n.device_class = dev_class;
  n.online = true;
  n.last_seen_ms = millis();
  n.managed = true;
  n.fail_slow = 0;
  n.state = ST_ONLINE;
  n.known = (slow_nvs_get_addr(uid) != 0);

  uint8_t stored_addr = slow_nvs_get_addr(uid);
  uint8_t desired_addr = (stored_addr != 0) ? stored_addr : cur_addr;

  if (desired_addr != 0 && slow_addr_in_use_online(desired_addr, idx_uid)) {
    uint8_t a2 = alloc_addr_online(idx_uid);
    if (a2 == 0) a2 = alloc_addr();
    DBG.printf("[SLOW] ADDR CONFLICT uid=%s want=0x%02X -> will assign 0x%02X\n", uid_s, desired_addr, a2);

    n.addr = 0;
    need_assign_specific[idx_uid] = true;
    need_assign_addr[idx_uid] = a2;
    mgmt_request = true;
    return true;
  }

  if (desired_addr == 0) {
    DBG.printf("[SLOW] ANN uid=%s class=0x%02X addr=0 -> pending\n", uid_s, dev_class);
    n.addr = 0;
    need_assign[idx_uid] = true;
    mgmt_request = true;
    return true;
  }

  if (cur_addr != desired_addr) {
    DBG.printf("[SLOW] %s reassign uid=%s cur=0x%02X -> 0x%02X\n",
                  (stored_addr != 0) ? "KNOWN" : "NEW",
                  uid_s, cur_addr, desired_addr);
    need_assign_specific[idx_uid] = true;
    need_assign_addr[idx_uid] = desired_addr;
    mgmt_request = true;
    return true;
  }

  n.addr = cur_addr;
  slow_nvs_set_addr(uid, cur_addr);
  DBG.printf("[SLOW] ANN uid=%s class=0x%02X addr=0x%02X -> ONLINE\n", uid_s, dev_class, cur_addr);
  return true;
}

static void start_descr(uint8_t node_i) {
  NodeInfo& n = nodes[node_i];
  if (n.addr == 0) return;

  tx.type = TX_DESCR;
  tx.addr = n.addr;
  tx.node_index = node_i;
  tx.seq = g_seq++;
  tx.retry = 0;
  tx.deadline_ms = millis() + 300;
  send_frame(MSG_DESCR_REQ, 0x00, n.addr, ADDR_MASTER, tx.seq, nullptr, 0);
}

static void start_read(uint8_t node_i) {
  NodeInfo& n = nodes[node_i];
  if (n.addr == 0) return;

  tx.type = TX_READ;
  tx.addr = n.addr;
  tx.node_index = node_i;
  tx.seq = g_seq++;
  tx.retry = 0;
  tx.deadline_ms = millis() + 300;
  uint8_t pay[1] = {0};
  send_frame(MSG_READ_REQ, 0x00, n.addr, ADDR_MASTER, tx.seq, pay, sizeof(pay));
}

static bool parse_descr_resp(NodeInfo& n, const uint8_t* raw, size_t ) {
  uint16_t plen = (uint16_t)raw[7] | ((uint16_t)raw[8] << 8);
  if (plen < (uint16_t)(6 + 1 + 1 + 1)) return false;

  const uint8_t* p = &raw[9];
  uint8_t uid[6]; memcpy(uid, p, 6); p += 6;
  uint8_t dev_class = *p++;
  uint8_t ver = *p++;
  uint8_t ch_count = *p++;

  memcpy(n.uid, uid, 6);
  n.device_class = dev_class;
  n.descr_ver = ver;
  n.ch_count = (ch_count > 64) ? 64 : ch_count;

  for (int i = 0; i < n.ch_count; i++) {
    if (ver >= 3) {
      if ((p - &raw[9]) + 11 > (int)plen) break;
      n.ch[i].ch_id = *p++;
      n.ch[i].dtype = *p++;               
      n.ch[i].unit  = *p++;
      n.ch[i].scale = (uint16_t)p[0] | ((uint16_t)p[1] << 8); p += 2;
      p += 6; 
      if (n.ch[i].scale == 0) n.ch[i].scale = 1;
    } else {
      if ((p - &raw[9]) + 5 > (int)plen) break;
      n.ch[i].ch_id = *p++;
      n.ch[i].dtype = *p++;
      n.ch[i].scale = (uint16_t)p[0] | ((uint16_t)p[1] << 8); p += 2;
      n.ch[i].unit  = *p++;
      if (n.ch[i].scale == 0) n.ch[i].scale = 1;
    }
  }
  return true;
}

static void parse_read_resp(NodeInfo& n, const uint8_t* raw, size_t ) {
  uint16_t plen = (uint16_t)raw[7] | ((uint16_t)raw[8] << 8);
  if (plen < 3) return;

  const uint8_t* p = &raw[9];
  uint16_t status = (uint16_t)p[0] | ((uint16_t)p[1] << 8);
  uint8_t count = p[2];
  p += 3;

  uint8_t ch_ids[60];
  uint16_t scales[60];
  uint8_t  units[60];
  int32_t values[60];
  uint8_t ncount = 0;

  for (uint8_t i = 0; i < count; i++) {
    if ((p - &raw[9]) + 5 > (int)plen) break;
    uint8_t ch_id = *p++;
    int32_t v = (int32_t)p[0] | ((int32_t)p[1] << 8) | ((int32_t)p[2] << 16) | ((int32_t)p[3] << 24);
    p += 4;

    if (ncount < 60) {
      ch_ids[ncount] = ch_id;
      scales[ncount] = find_scale(n, ch_id);
      units[ncount]  = find_unit(n, ch_id);
      values[ncount] = v;
      ncount++;
    }
  }

  usb_emit_slow_snapshot(n.uid, n.addr, status, ch_ids, values, scales, units, ncount);

  if (logcfg::SLOW_PRINT_DATA) {
    char uid_s[13]; uid_to_str(n.uid, uid_s);
    DBG.printf("[SLOW] SNAP uid=%s addr=0x%02X status=0x%04X count=%u\n", uid_s, n.addr, status, ncount);
  }
}

static bool assign_addr_to_uid_specific(const uint8_t uid[6], int idx, const char* uid_s, uint8_t new_addr) {
  if (new_addr == 0 || new_addr == ADDR_MASTER || new_addr == ADDR_BROADCAST) new_addr = alloc_addr();

  uint8_t pay[7];
  memcpy(pay, uid, 6);
  pay[6] = new_addr;

  tx.type = TX_ASSIGN;
  tx.addr = new_addr;
  tx.node_index = (uint8_t)idx;
  tx.seq = g_seq++;
  tx.deadline_ms = millis() + 400;
  tx.retry = 0;

  send_frame(MSG_ASSIGN_ADDR, 0x00, ADDR_BROADCAST, ADDR_MASTER, tx.seq, pay, sizeof(pay));
  DBG.printf("[SLOW] Assign start uid=%s -> addr=0x%02X\n", uid_s, new_addr);
  return true;
}

static bool assign_addr_to_uid(const uint8_t uid[6], int idx, const char* uid_s) {
  uint8_t a = alloc_addr_online(idx);
  if (a == 0) a = alloc_addr();
  return assign_addr_to_uid_specific(uid, idx, uid_s, a);
}

static void slow_pump_and_dispatch() {
  uint8_t raw[256]; size_t len = 0;
  while (recv_frame(raw, sizeof(raw), &len)) {
    if (len < 11) continue;

    if (handle_announce_runtime(raw, len)) continue;

    if (raw[0] != VER_1) continue;
    if (raw[3] != ADDR_MASTER) continue;

    uint8_t msg = raw[1];
    uint16_t rseq = (uint16_t)raw[5] | ((uint16_t)raw[6] << 8);

    if (tx.type == TX_NONE) continue;

    if (tx.type == TX_ASSIGN) {
      if (rseq != tx.seq) continue;
      if (msg == MSG_ADDR_CONFIRM) {
        uint16_t plen = (uint16_t)raw[7] | ((uint16_t)raw[8] << 8);
        if (plen < 1) continue;
        if (raw[9] != tx.addr) continue;

        NodeInfo& n = nodes[tx.node_index];
        n.addr = tx.addr;
        n.online = true;
        n.last_seen_ms = millis();
        slow_join_ok++;

        slow_nvs_set_addr(n.uid, n.addr);

        DBG.printf("[SLOW] Assigned ok uid=%02X%02X.. -> addr=0x%02X\n", n.uid[5], n.uid[4], n.addr);

        tx.type = TX_NONE;
        n.ch_count = 0;
        start_descr(tx.node_index);
        return;
      }
      continue;
    }

    if (tx.type == TX_DESCR && msg == MSG_DESCR_RESP && raw[4] == tx.addr) {
      NodeInfo& n = nodes[tx.node_index];
      if (parse_descr_resp(n, raw, len)) {
        n.online = true;
        n.last_seen_ms = millis();
        n.fail_slow = 0;
        n.state = ST_ONLINE;

        char uid_s[13]; uid_to_str(n.uid, uid_s);
        DBG.printf("[SLOW] DESCR ok addr=0x%02X class=0x%02X ver=%u uid=%s ch=%u\n",
                   n.addr, n.device_class, n.descr_ver, uid_s, n.ch_count);

        usb_emit_slow_descr(n);
      }
      tx.type = TX_NONE;
      return;
    }

    if (tx.type == TX_READ && msg == MSG_READ_RESP && raw[4] == tx.addr) {
      NodeInfo& n = nodes[tx.node_index];
      parse_read_resp(n, raw, len);
      n.online = true;
      n.last_seen_ms = millis();
      n.fail_slow = 0;
      n.state = ST_ONLINE;
      tx.type = TX_NONE;
      return;
    }
  }
}

static void preload_known_nodes() {
  uint8_t uids[MAX_KNOWN_UIDS][6];
  uint8_t count = 0;
  if (!reg_load_uids(uids, count) || count == 0) return;

  for (uint8_t i = 0; i < count; i++) {
    uint8_t a = slow_nvs_get_addr(uids[i]);
    if (a == 0) continue;

    int idx = find_node_by_uid(uids[i]);
    if (idx < 0) {
      if (node_count >= (int)(sizeof(nodes) / sizeof(nodes[0]))) continue;
      idx = node_count++;
      memset(&nodes[idx], 0, sizeof(NodeInfo));
      memcpy(nodes[idx].uid, uids[i], 6);
      last_logged_state[idx] = ST_UNASSIGNED;
    }

    nodes[idx].addr = a;
    nodes[idx].known = true;
    nodes[idx].managed = true;
    nodes[idx].state = ST_SUSPECT;
    nodes[idx].online = false;
    nodes[idx].last_seen_ms = 0;
    nodes[idx].fail_slow = 0;
  }
}

void slow_init() {
  delay(200);

  assert_gpio_ok(RS485_RX, "RS485_RX");
  assert_gpio_output_ok(RS485_TX, "RS485_TX");

  RS485.begin(SLOW_BAUD, SERIAL_8N1, RS485_RX, RS485_TX);

  preload_known_nodes();

  DBG.println("SLOW_MASTER boot (ESP32-S3, non-block)");
  DBG.println("[SLOW] DISCOVERY: open window...");
  open_discovery_window(1200, 250, DISC_FLAG_FORCE_HELLO_ALL);

  uint32_t t0 = millis();
  while (millis() - t0 < 1300) {
    slow_pump_and_dispatch();
    delay(1);
  }

  DBG.printf("[SLOW] DISCOVERY done. node_count=%d\n", node_count);

  for (int i = 0; i < node_count; i++) {
    if (nodes[i].addr == 0) {
      char uid_s[13]; uid_to_str(nodes[i].uid, uid_s);
      assign_addr_to_uid(nodes[i].uid, i, uid_s);
      break;
    }
  }
}

void tick() {
  uint32_t now = millis();
  slow_pump_and_dispatch();

  if (mgmt_active) {
    if ((int32_t)(now - mgmt_until_ms) < 0) return;
    mgmt_active = false;
  }

  bool time_to_disc = (now - last_disc_ms >= DISCOVERY_PERIOD_MS);
  if ((time_to_disc || mgmt_request) && tx.type == TX_NONE) {
    mgmt_request = false;
    last_disc_ms = now;
    open_discovery_window(DISC_WINDOW_MS, DISC_RAND_MS, DISC_FLAG_FORCE_HELLO_ALL);

    mgmt_active   = true;
    mgmt_until_ms = now + DISC_WINDOW_MS + DISC_RAND_MS + DISC_GUARD_MS;
    return;
  }

  if (tx.type == TX_NONE) {
    for (int i = 0; i < node_count; i++) {
      if (need_assign_specific[i]) {
        need_assign_specific[i] = false;
        char uid_s2[13]; uid_to_str(nodes[i].uid, uid_s2);
        assign_addr_to_uid_specific(nodes[i].uid, i, uid_s2, need_assign_addr[i]);
        return;
      }
      if (need_assign[i]) {
        need_assign[i] = false;
        char uid_s2[13]; uid_to_str(nodes[i].uid, uid_s2);
        assign_addr_to_uid(nodes[i].uid, i, uid_s2);
        return;
      }
    }
  }

  for (int i = 0; i < node_count; i++) {
    NodeInfo& n = nodes[i];
    if (n.state == ST_ONLINE || n.state == ST_SUSPECT) {
      uint32_t age_ms = now - n.last_seen_ms;

      if ((int32_t)age_ms > (int32_t)OFFLINE_AFTER_MS) {
        if (n.state != ST_OFFLINE) {
          n.state  = ST_OFFLINE;
          n.online = false;

          if (last_logged_state[i] != ST_OFFLINE) {
            last_logged_state[i] = ST_OFFLINE;
            char uid_s[13]; uid_to_str(n.uid, uid_s);
            g_node_state_changes++;
            if (logcfg::LOG_STATE_CHANGES) {
              DBG.printf("[SLOW] %s addr=0x%02X uid=%s age=%lu ms\n",
                            state_to_str(n.state), n.addr, uid_s, (unsigned long)age_ms);
            }
          }
        }
      }
    }
  }

  if (tx.type != TX_NONE) {
    if ((int32_t)(now - tx.deadline_ms) >= 0) {
      if ((tx.type == TX_READ || tx.type == TX_DESCR) && tx.node_index < (uint8_t)node_count) {
        NodeInfo& n = nodes[tx.node_index];

        if (tx.retry < 1) {
          tx.retry++;
          tx.seq = g_seq++;
          tx.deadline_ms = now + 300;
          if (tx.type == TX_DESCR) {
            send_frame(MSG_DESCR_REQ, 0x00, n.addr, ADDR_MASTER, tx.seq, nullptr, 0);
          } else {
            uint8_t pay[1] = {0};
            send_frame(MSG_READ_REQ, 0x00, n.addr, ADDR_MASTER, tx.seq, pay, sizeof(pay));
          }
          return;
        }

        if (n.fail_slow < 255) n.fail_slow++;
        slow_poll_timeout++;
        if (n.fail_slow >= 3) {
          n.state = ST_SUSPECT;
          n.online = false;
        }
        tx.type = TX_NONE;
        return;
      }

      if (tx.type == TX_ASSIGN && tx.retry < 2) {
        tx.retry++;
        NodeInfo& n = nodes[tx.node_index];
        uint8_t pay[7];
        memcpy(pay, n.uid, 6);
        pay[6] = tx.addr;
        tx.seq = g_seq++;
        tx.deadline_ms = now + 400;
        send_frame(MSG_ASSIGN_ADDR, 0x00, ADDR_BROADCAST, ADDR_MASTER, tx.seq, pay, sizeof(pay));
        return;
      } else {
        tx.type = TX_NONE;
        return;
      }
    }
    return; 
  }

  const uint32_t POLL_PERIOD_MS = 250;
  if (now - last_poll_ms < POLL_PERIOD_MS) return;
  last_poll_ms = now;

  if (node_count == 0) return;

  for (int k = 0; k < node_count; k++) {
    poll_idx = (uint8_t)((poll_idx + 1) % (uint8_t)node_count);
    NodeInfo& n = nodes[poll_idx];
    if (n.addr == 0) continue;

    if (n.ch_count == 0) start_descr(poll_idx);
    else start_read(poll_idx);
    break;
  }
}

void health_tick() {
  static uint32_t last_ms = 0;
  uint32_t now = millis();
  if (now - last_ms < 5000) return;
  last_ms = now;

  uint8_t on=0, known=0, managed=0;
  for (int i=0;i<node_count;i++){
    if (nodes[i].state != ST_UNASSIGNED) {
      if (nodes[i].state == ST_ONLINE) on++;
      if (nodes[i].known) known++;
      if (nodes[i].managed) managed++;
    }
  }

  DBG.printf("[HEALTH] SLOW on=%u known=%u managed=%u rx_ok=%lu crc=%lu cobs=%lu short=%lu toolong=%lu ovf=%lu poll_to=%lu join_ok=%lu/%lu state_changes=%lu\n",
                (unsigned)on,(unsigned)known,(unsigned)managed,
                (unsigned long)slow_rx_ok,(unsigned long)slow_err_crc,(unsigned long)slow_err_cobs,
                (unsigned long)slow_err_short,(unsigned long)slow_err_toolong,(unsigned long)slow_err_overflow,
                (unsigned long)slow_poll_timeout,(unsigned long)slow_join_ok,(unsigned long)slow_join_seen,
                (unsigned long)g_node_state_changes);
}

} 

static bool SLOW_HEALTH_LOG = true;
