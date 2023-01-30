#include <SPI.h>
#include <SD.h>
#include <DueTimer.h>

const int chipSelect = 10;
File dataFile;

void ISR_100Hz() {
  while (Serial1.available()) {
    char c = Serial1.read();
    SerialUSB.write(c);//Serial.parseFloat()
    dataFile.write(c);
  }
}

void setup() {
  // Open SerialUSB communications and wait for port to open:
  SerialUSB.begin(115200);
  Serial1.begin(115200);
  while (!SerialUSB) {
    ; // wait for SerialUSB port to connect. Needed for native USB port only
  }


  SerialUSB.print("Initializing SD card...");

  if (!SD.begin(chipSelect)) {
    SerialUSB.println("Card failed, or not present");
    while (1);
  }
  SerialUSB.println("card initialized.");

  Timer3.attachInterrupt(ISR_100Hz).start(5000);
  NVIC_SetPriority((IRQn_Type)TC3_IRQn, 14);
}

void loop() {
  unsigned long time = millis();
  SerialUSB.println("open_start");
  dataFile = SD.open("datalog.txt", FILE_WRITE);
  SerialUSB.println("open_end");

  if (dataFile) {
    SerialUSB.println("close_start");
    dataFile.close();
    SerialUSB.println("close_end");
    SerialUSB.print("SD_total:");
    SerialUSB.println(millis() - time);
  }
  else {
    SerialUSB.println("error opening datalog.txt");
  }
  delay(1000);
}
