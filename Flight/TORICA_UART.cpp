#include "TORICA_UART.h"
#include <Arduino.h>

int TORICA_UART::readUART() {
  while (serial->available() > 0) {
    buff[i_buff] = (char)serial->read();
    if (buff[i_buff] == '\n') {
      buff[i_buff] == '\0';
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
      i_buff = 0;
      return i_UART_data;
    } else {
      i_buff += 1;
    }
  }
  return 0;
}
