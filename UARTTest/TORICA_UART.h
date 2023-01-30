#include <Arduino.h>
class TORICA_UART {
public:
  TORICA_UART(Stream *_serial) {
    serial = _serial;
  }

  int readUART();
  float UART_data[10];

private:
  int i = 0;
  float pressure_hPa;
  float temperature_deg;
  float P_hight_m;
  float S_hight_m;
  char buff[30];
  char *p;
  Stream *serial;
};