#pragma once
#include <Arduino.h>
#include <Preferences.h>
#include <stdarg.h>
#include "driver/gpio.h"

enum MsgType : uint8_t {
  MSG_HELLO        = 0x03, 
  MSG_ASSIGN_ID    = 0x04, 
  MSG_ID_ACK       = 0x05, 
  MSG_DISC_OPEN    = 0x71, 
  MSG_SCHEDULE     = 0x73, 
  MSG_START_STREAM = 0x20,
};

enum DevType : uint8_t { DEV_UNKNOWN=0, DEV_MIC=1, DEV_VIB=2 };
enum StreamFmt : uint8_t { FMT_UNKNOWN=0, FMT_PCM16=1, FMT_XYZ16=2 };

#pragma pack(push, 1)
struct PacketHeader {
  uint8_t magic0, magic1, ver;
  uint8_t src, dst, type, flags;
  uint16_t seq;
  uint16_t len;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct HelloDescr {
  uint8_t uid[6];
  uint8_t dev_type;
  uint16_t fs_hz;
  uint8_t fmt;
  uint8_t ch_or_axes;
  uint8_t block_ms;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct SchedEntry {
  uint8_t uid[6];
  uint8_t id;
  uint32_t offset_us;
  uint8_t period_cycles;
  uint8_t stream_sof;
};
#pragma pack(pop)

struct NodeInfo {
  uint8_t uid[6];
  DevType dev;
  uint16_t fs;
  StreamFmt fmt;
  uint8_t ch_axes;
  uint8_t block_ms;
  uint8_t assigned_id;
  bool id_acked;
  bool selected;
};

extern HardwareSerial DBG;
