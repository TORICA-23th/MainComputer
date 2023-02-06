#include <Arduino.h>

class TORICA_ICS {
  public:
    TORICA_ICS(Stream *_serial) {
      serial = _serial;
    }

    int read_Angle();

  private:
    int angle;
    Stream *serial;
};
