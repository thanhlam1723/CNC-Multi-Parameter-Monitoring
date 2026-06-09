#pragma once
#include "common.h"

namespace usbmux {
  void begin(Stream& usb);
  bool enqueue(uint8_t type, const uint8_t* data, uint16_t len, bool fast_prio);

  inline bool fast_chunk(const uint8_t* data, uint16_t len) { return enqueue(0x01, data, len, true); }
  inline bool slow_snapshot(const uint8_t* data, uint16_t len) { return enqueue(0x02, data, len, false); }

  void flush();

  uint32_t dropped_fast();
  uint32_t dropped_slow();
  uint32_t queued_fast_bytes();
  uint32_t queued_slow_bytes();
}
