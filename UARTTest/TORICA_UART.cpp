#include "TORICA_UART.h"
#include <Arduino.h>

int TORICA_UART::readUART() {
  if (serial->available() > 0) {
    buff[i] = serial->read();
    if (buff[i] == '\n') {
      buff[i] == '\0';
      UART_data[0] = atof(strtok(buff, ","));
      int i_UART_data;
      for (i_UART_data = 1; true; i_UART_data++) {
        p = strtok(NULL, ",");
        if (p != NULL) {
          UART_data[i_UART_data] = atof(p);
        } else {
          break;
        }
      }
      i = 0;
      return i_UART_data;
    } else {
      i += 1;
    }
  }
  return 0;
}