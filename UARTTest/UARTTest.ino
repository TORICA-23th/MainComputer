#include "TORICA_UART.h"

TORICA_UART Alt_UART(&Serial3);

void setup() {
  SerialUSB.begin(115200);
  Serial3.begin(115200);
}

void loop() {
  int readnum = Alt_UART.readUART();
  for (int j = 0; j < readnum; j++) {
    SerialUSB.println(Alt_UART.UART_data[j]);
  }
   Serial3.println("Hello");
}