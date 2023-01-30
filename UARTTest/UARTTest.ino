#include "TORICA_UART.h"

TORICA_UART Alt_UART(&Serial1);

void setup() {
  SerialUSB.begin(115200);
  Serial1.begin(115200);
}

void loop() {
  int readnum = Alt_UART.readUART();
  for (int j = 0; j < readnum; j++) {
    SerialUSB.println(Alt_UART.UART_data[j]);
  }
}