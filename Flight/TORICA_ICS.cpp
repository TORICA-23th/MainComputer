#include "TORICA_ICS.h"
#include <Arduino.h>

int TORICA_ICS::read_Angle() {
  if (serial->available() >= 3) {
    char c = (char)serial->read();
    if (c == 128) {
      uint16_t bit1 = (char)serial->read();
      uint16_t bit2 = (char)serial->read();
      angle = bit1 * 128 + bit2;
      return angle;
    }
  }
  return 0;
}
