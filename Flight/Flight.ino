#include <DueTimer.h>
bool skip_ISR = false;

#include <SPI.h>
#include <SD.h>
const int cs_SD = 10;
char fileName[16];
File dataFile;
String SD_buf[2];
int SD_buf_index = 0;

#include "TORICA_UART.h"
TORICA_UART Under_UART(&Serial1);
TORICA_UART Air_UART(&Serial2);


void ISR_200Hz() {
  if (skip_ISR) {
    skip_ISR = false;
    return;
  }
  unsigned long time = micros();
  unsigned long time_ms = millis();
  //UnderSide
  int readnum = Under_UART.readUART();
  if (readnum == 4) {
    SD_buf[SD_buf_index].concat("UNDER,");
    SD_buf[SD_buf_index].concat(time_ms);
    for (int j = 0; j < readnum; j++) {
      //SerialUSB.println(Under_UART.UART_data[j]);
      SD_buf[SD_buf_index].concat(",");
      SD_buf[SD_buf_index].concat(Under_UART.UART_data[j]);
    }
    SD_buf[SD_buf_index].concat("\n");
  }

  //AirData



  //SerialUSB.print("SD_buf_index:");
  //SerialUSB.println(SD_buf_index);

  if (micros() - time > 3000) { //MAX5000=200Hz
    skip_ISR = true;
  }
  SerialUSB.print("ISR_us:");
  SerialUSB.println(micros() - time);
}

void setup() {

  Serial1.begin(115200);
  Serial2.begin(115200);

  SerialUSB.begin(115200);
  while (!SerialUSB) {
    ; // wait for SerialUSB port to connect. Needed for native USB port only
  }

  pinMode(LED_BUILTIN, OUTPUT);
  SerialUSB.print("Initializing SD card...");
  if (!SD.begin(cs_SD)) {
    SerialUSB.println("Card failed, or not present");
    while (1);
  }
  String s;
  int fileNum = 0;
  while (1) {
    s = "LOG";
    if (fileNum < 10) {
      s += "00";
    } else if (fileNum < 100) {
      s += "0";
    }
    s += fileNum;
    s += ".CSV";
    s.toCharArray(fileName, 16);
    if (!SD.exists(fileName)) break;
    fileNum++;
  }
  SerialUSB.println("card initialized.");

  Timer3.attachInterrupt(ISR_200Hz).start(5000);
  NVIC_SetPriority((IRQn_Type)SysTick_IRQn, 14);
  NVIC_SetPriority((IRQn_Type)TC3_IRQn, 15);
}

void loop() {

  digitalWrite(LED_BUILTIN, HIGH);

  unsigned long time = millis();

  int previous_index = SD_buf_index;
  SD_buf_index = (SD_buf_index + 1) % 2;

  SerialUSB.println("open_start");
  dataFile = SD.open(fileName, FILE_WRITE);
  SerialUSB.println("open_end");
  if (dataFile) {
    dataFile.println(SD_buf[previous_index]);

    SerialUSB.println("close_start");
    dataFile.close();
    SerialUSB.println("close_end");
    SerialUSB.print("SD_total:");
    SerialUSB.println(millis() - time);
  }
  else {
    SerialUSB.println("error opening file");
  }
  digitalWrite(LED_BUILTIN, LOW);

  SD_buf[previous_index] = "";
  delay(1000);
}
