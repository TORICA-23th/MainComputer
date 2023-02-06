#include "TORICA_SD.h"
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>

bool TORICA_SD::begin() {
  SerialUSB.print("Initializing SD card...");
  if (!SD.begin(cs_SD)) {
    SerialUSB.println("Card failed, or not present");
    return false;
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
  return true;
}

void TORICA_SD::add_str(String str) {
  if (SDisActive) {
    SD_buf[SD_buf_index].concat(str);
  }
}

void TORICA_SD::flash() {
  digitalWrite(LED_BUILTIN, HIGH);
  unsigned long SD_time = millis();
  int previous_index = SD_buf_index;
  SD_buf_index = (SD_buf_index + 1) % 2;

  SerialUSB.println("open_start");
  dataFile = SD.open(fileName, FILE_WRITE);
  SerialUSB.println("open_end");
  if (dataFile) {
    dataFile.print(SD_buf[previous_index]);
    SerialUSB.println("close_start");
    dataFile.close();
    SerialUSB.println("close_end");
    SerialUSB.print("SD_total:");
    SerialUSB.println(millis() - SD_time);
  }
  else {
    SerialUSB.println("error opening file");
    SDisActive = false;
    SD.end();
  }
  digitalWrite(LED_BUILTIN, LOW);

  SD_buf[previous_index] = "";
  //https://www.arduino.cc/reference/en/language/variables/data-types/string/functions/reserve/
  //SD_buf[previous_index].reserve(sizeof(char) * 32768);
}
