#include <DueTimer.h>
#include "TORICA_UART.h"

char incomingByte = 0;

TORICA_UART Alt_UART(&Serial1);

void ISR_200Hz() {
  int readnum = Alt_UART.readUART();
  for (int j = 0; j < readnum; j++) {
    SerialUSB.println(Alt_UART.UART_data[j]);
  }
}

void setup() {
  SerialUSB.begin(115200);
  Serial1.begin(115200);

  NVIC_SetPriority((IRQn_Type)SysTick_IRQn, 14);
  NVIC_SetPriority((IRQn_Type)TC1_IRQn, 15);

  Timer3.attachInterrupt(ISR_200Hz);
  Timer3.start(5000);
}

void loop() {

  while (1) {
    // I'm stuck in here! help me...
  }
}