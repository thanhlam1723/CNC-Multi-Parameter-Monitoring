#pragma once
#include "common.h"

namespace slowbus {
  extern Stream& DATA; // USB CDC
  extern Stream& DBG;  // UART0
  void slow_init();
  void tick();
  void health_tick();
}
