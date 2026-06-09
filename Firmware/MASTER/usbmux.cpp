#include "usbmux.h"

#ifndef USBMUX_FAST_Q_SIZE
#define USBMUX_FAST_Q_SIZE 16384
#endif
#ifndef USBMUX_SLOW_Q_SIZE
#define USBMUX_SLOW_Q_SIZE 4096
#endif

static Stream* g_usb = nullptr;

static uint8_t q_fast[USBMUX_FAST_Q_SIZE];
static uint8_t q_slow[USBMUX_SLOW_Q_SIZE];
static volatile uint32_t wf=0, rf=0;
static volatile uint32_t ws=0, rs=0;

static uint32_t drop_fast=0, drop_slow=0;
static uint16_t seq_fast=1, seq_slow=1;

static uint16_t crc16_modbus(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int b = 0; b < 8; b++) {
      if (crc & 1) crc = (crc >> 1) ^ 0xA001;
      else         crc = (crc >> 1);
    }
  }
  return crc;
}

static size_t cobs_encode(const uint8_t* in, size_t len, uint8_t* out, size_t out_max) {
  if (out_max == 0) return 0;
  size_t read_index = 0;
  size_t write_index = 1;
  size_t code_index = 0;
  uint8_t code = 1;

  while (read_index < len) {
    if (in[read_index] == 0) {
      out[code_index] = code;
      code = 1;
      code_index = write_index++;
      if (write_index >= out_max) return 0;
      read_index++;
    } else {
      out[write_index++] = in[read_index++];
      if (write_index >= out_max) return 0;
      code++;
      if (code == 0xFF) {
        out[code_index] = code;
        code = 1;
        code_index = write_index++;
        if (write_index >= out_max) return 0;
      }
    }
  }
  out[code_index] = code;
  return write_index;
}

static inline uint32_t ring_used(uint32_t w, uint32_t r, uint32_t cap) {
  return (w >= r) ? (w - r) : (cap - (r - w));
}
static inline uint32_t ring_free(uint32_t w, uint32_t r, uint32_t cap) {
  return cap - 1 - ring_used(w, r, cap);
}

static bool ring_push(uint8_t* ring, uint32_t cap, volatile uint32_t& w, volatile uint32_t& r,
                      const uint8_t* data, uint32_t n) {
  if (n == 0) return true;
  uint32_t free = ring_free(w, r, cap);
  if (n > free) return false;

  uint32_t tail = cap - w;
  if (n <= tail) {
    memcpy(&ring[w], data, n);
    w = (w + n) % cap;
  } else {
    memcpy(&ring[w], data, tail);
    memcpy(&ring[0], data + tail, n - tail);
    w = (n - tail);
  }
  return true;
}

static uint32_t ring_peek_contig(uint32_t w, uint32_t r, uint32_t cap) {
  if (w >= r) return (w - r);
  return (cap - r);
}

static void ring_drop(uint32_t cap, volatile uint32_t& r, uint32_t n) {
  r = (r + n) % cap;
}

namespace usbmux {

void begin(Stream& usb) { g_usb = &usb; }

bool enqueue(uint8_t type, const uint8_t* data, uint16_t len, bool fast_prio) {
  if (!g_usb) return false;

  if (len > 2048) len = 2048;

  uint8_t tmp[1 + 2 + 2 + 4 + 2048 + 2];
  size_t i = 0;
  tmp[i++] = type;

  uint16_t seq = fast_prio ? seq_fast++ : seq_slow++;
  tmp[i++] = (uint8_t)(seq & 0xFF);
  tmp[i++] = (uint8_t)(seq >> 8);

  tmp[i++] = (uint8_t)(len & 0xFF);
  tmp[i++] = (uint8_t)(len >> 8);

  uint32_t t_us = (uint32_t)micros();
  tmp[i++] = (uint8_t)(t_us & 0xFF);
  tmp[i++] = (uint8_t)((t_us >> 8) & 0xFF);
  tmp[i++] = (uint8_t)((t_us >> 16) & 0xFF);
  tmp[i++] = (uint8_t)((t_us >> 24) & 0xFF);

  if (len && data) {
    memcpy(&tmp[i], data, len);
    i += len;
  }

  uint16_t crc = crc16_modbus(tmp, i);
  tmp[i++] = (uint8_t)(crc & 0xFF);
  tmp[i++] = (uint8_t)(crc >> 8);

  uint8_t enc[1 + 2 + 2 + 4 + 2048 + 2 + 16]; 
  size_t enc_len = cobs_encode(tmp, i, enc, sizeof(enc));
  if (enc_len == 0) return false;

  bool ok;
  if (fast_prio) {
    ok = ring_push(q_fast, USBMUX_FAST_Q_SIZE, wf, rf, enc, (uint32_t)enc_len) &&
         ring_push(q_fast, USBMUX_FAST_Q_SIZE, wf, rf, (const uint8_t*)"\x00", 1);
    if (!ok) { drop_fast++; }
  } else {
    ok = ring_push(q_slow, USBMUX_SLOW_Q_SIZE, ws, rs, enc, (uint32_t)enc_len) &&
         ring_push(q_slow, USBMUX_SLOW_Q_SIZE, ws, rs, (const uint8_t*)"\x00", 1);
    if (!ok) { drop_slow++; }
  }
  return ok;
}

void flush() {
  if (!g_usb) return;

  auto drain = [&](uint8_t* ring, uint32_t cap, volatile uint32_t& w, volatile uint32_t& r) {
    uint32_t used = ring_used(w, r, cap);
    if (!used) return;

    int can = 0;

    can = (int)g_usb->availableForWrite();
    if (can <= 0) return;

    uint32_t n = (used < (uint32_t)can) ? used : (uint32_t)can;
    if (n > 1024) n = 1024;

    uint32_t cont = ring_peek_contig(w, r, cap);
    uint32_t sendn = (n < cont) ? n : cont;
    if (sendn) {
      g_usb->write(&ring[r], sendn);
      ring_drop(cap, r, sendn);
    }
  };

  for (int k = 0; k < 4; k++) {
    drain(q_fast, USBMUX_FAST_Q_SIZE, wf, rf);
    if (ring_used(wf, rf, USBMUX_FAST_Q_SIZE) == 0) break;
  }

  drain(q_slow, USBMUX_SLOW_Q_SIZE, ws, rs);
}

uint32_t dropped_fast() { return drop_fast; }
uint32_t dropped_slow() { return drop_slow; }
uint32_t queued_fast_bytes(){ return ring_used(wf, rf, USBMUX_FAST_Q_SIZE); }
uint32_t queued_slow_bytes(){ return ring_used(ws, rs, USBMUX_SLOW_Q_SIZE); }

}
