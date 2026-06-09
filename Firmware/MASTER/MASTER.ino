#include "common.h"
#include "usbmux.h"
#include "fastbus.h"
#include "slowbus.h"

HardwareSerial DBG(0);

static bool SLOW_HEALTH_LOG = true;

void setup() {
  DBG.begin(115200);
  delay(100);
  DBG.println("\n[COMBINED] boot (DBG=UART0, DATA=USB CDC)");

  Serial.begin(0);
  delay(50);
  usbmux::begin(Serial);

  slowbus::slow_init();
  DBG.println("[COMBINED] slowbus init OK");

  fastbus::fast_init();
  DBG.println("[COMBINED] fastbus init OK");
}

void loop() {
  fastbus::fast_tick();
  usbmux::flush();

  slowbus::tick();
  if (SLOW_HEALTH_LOG) slowbus::health_tick();

  usbmux::flush();
}
