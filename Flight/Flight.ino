/*
SDをクラスにまとめる
操舵量を取得する
GPSを試す
*/
#include <DueTimer.h>

#include <SPI.h>
#include <SD.h>
const int cs_SD = 10;
char fileName[16];
File dataFile;
bool SDisActive = false;
String SD_buf[2];
int SD_buf_index = 0;

#include "TORICA_UART.h"
TORICA_UART Under_UART(&Serial1);
TORICA_UART Air_UART(&Serial2);

#include <TinyGPSPlus.h>
TinyGPSPlus gps;
#define SerialGPS Serial

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);

#include <Adafruit_DPS310.h>
Adafruit_DPS310 dps;
sensors_event_t temp_event, pressure_event;

void ISR_UART_500Hz() {
  unsigned long time = micros();

  //UnderSide
  int readnum = Under_UART.readUART();
  if (readnum == 4) {
    String SD_Under = "";
    SD_Under.concat("UNDER,");
    SD_Under.concat(millis());
    for (int j = 0; j < readnum; j++) {
      //SerialUSB.println(Under_UART.UART_data[j]);
      SD_Under.concat(",");
      SD_Under.concat(Under_UART.UART_data[j]);
    }
    SD_Under.concat("\n");
    add_str_SD(SD_Under);
  }

  //AirData
  //ToDo

  //GPS
  while (SerialGPS.available() > 0) {
    if (gps.encode(SerialGPS.read())) {
      String SD_GPS = "";
      SD_GPS.concat("GPS,");
      SD_GPS.concat(millis());

      SD_GPS.concat(",");
      SD_GPS.concat(gps.time.hour());
      SD_GPS.concat(",");
      SD_GPS.concat(gps.time.minute());
      SD_GPS.concat(",");
      SD_GPS.concat(gps.time.second());
      SD_GPS.concat(".");
      SD_GPS.concat(gps.time.centisecond());

      SD_GPS.concat(",");
      SD_GPS.concat(String(gps.location.lat(), 6));
      SD_GPS.concat(",");
      SD_GPS.concat(String(gps.location.lng(), 6));

      SD_GPS.concat(",");
      SD_GPS.concat(String(gps.altitude.meters(), 2));

      SD_GPS.concat("\n");
      add_str_SD(SD_GPS);
    }
  }

  //DEBUG
  if (micros() - time > 1900) { //MAX2000=500Hz
    SerialUSB.print("ISR500Hz_overrun!!!");
  }
  //SerialUSB.print("ISR_us:");
  //SerialUSB.println(micros() - time);
}

void ISR_I2C0_100Hz() {
  unsigned long time = micros();

  String SD_IMU = "";
  SD_IMU.concat("IMU,");
  SD_IMU.concat(millis());
  SD_IMU.concat(",");
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  SD_IMU.concat(accel.x());
  SD_IMU.concat(",");
  SD_IMU.concat(accel.y());
  SD_IMU.concat(",");
  SD_IMU.concat(accel.z());
  SD_IMU.concat(",");
  /*imu::Vector<3> magnet = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    SD_IMU.concat(magnet.x());
    SD_IMU.concat(",");
    SD_IMU.concat(magnet.y());
    SD_IMU.concat(",");
    SD_IMU.concat(magnet.z());
    SD_IMU.concat(",");
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    SD_IMU.concat(gyro.x());
    SD_IMU.concat(",");
    SD_IMU.concat(gyro.y());
    SD_IMU.concat(",");
    SD_IMU.concat(gyro.z());
    SD_IMU.concat(",");
    imu::Vector<3> ground_acc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    SD_IMU.concat(ground_acc.x());
    SD_IMU.concat(",");
    SD_IMU.concat(ground_acc.y());
    SD_IMU.concat(",");
    SD_IMU.concat(ground_acc.z());
    SD_IMU.concat(",");*/
  imu::Quaternion quat = bno.getQuat();
  SD_IMU.concat(quat.w());
  SD_IMU.concat(",");
  SD_IMU.concat(quat.x());
  SD_IMU.concat(",");
  SD_IMU.concat(quat.y());
  SD_IMU.concat(",");
  SD_IMU.concat(quat.z());
  SD_IMU.concat("\n");
  add_str_SD(SD_IMU);

  String SD_PRESSURE = "";
  if (dps.temperatureAvailable() && dps.pressureAvailable()) {
    dps.getEvents(&temp_event, &pressure_event);
    SD_PRESSURE.concat("PRESSURE,");
    SD_PRESSURE.concat(millis());
    SD_PRESSURE.concat(",");
    SD_PRESSURE.concat(pressure_event.pressure);
    SD_PRESSURE.concat(",");
    SD_PRESSURE.concat(temp_event.temperature);
    SD_PRESSURE.concat("\n");
  }
  add_str_SD(SD_PRESSURE);

  if (micros() - time > 9900) { //MAX10000=100Hz
    SerialUSB.print("ISR100Hz_overrun!!!");
  }
  //SerialUSB.print("ISR_us:");
  //SerialUSB.println(micros() - time);
}

void setup() {
  //delay for flash
  delay(3000);

  Serial1.begin(115200);
  Serial2.begin(115200);
  SerialGPS.begin(115200);

  SerialUSB.begin(115200);
  /*
    while (!SerialUSB) {
    ; // wait for SerialUSB port to connect. Needed for native USB port only
    }*/

  pinMode(LED_BUILTIN, OUTPUT);
  SDisActive = begin_SD();

  Wire.setClock(400000);
  if (! dps.begin_I2C()) {             // Can pass in I2C address here
    //if (! dps.begin_SPI(DPS310_CS)) {  // If you want to use SPI
    SerialUSB.println("Failed to find DPS");
    while (1) yield();
  }
  dps.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
  dps.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);
  SerialUSB.println("DPS OK!");
  
  if (!bno.begin())
  {
    SerialUSB.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }

  //for CALLOUT
  Wire1.setClock(400000);
  //ToDo


  Timer3.attachInterrupt(ISR_I2C0_100Hz).start(10000);
  Timer4.attachInterrupt(ISR_UART_500Hz).start(2000);
  NVIC_SetPriority((IRQn_Type)SysTick_IRQn, 13);
  NVIC_SetPriority((IRQn_Type)TC3_IRQn, 14); //https://github.com/ivanseidel/DueTimer/blob/master/DueTimer.cpp#L17
  NVIC_SetPriority((IRQn_Type)TC4_IRQn, 15); //https://github.com/ivanseidel/DueTimer/blob/master/DueTimer.cpp#L18
}

void loop() {
  if (SDisActive) {
    flash_SD();
  } else {
    SDisActive = begin_SD();
  }
  delay(1000);//millisで実装
  callout_altitude();
}

bool begin_SD() {
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

void add_str_SD(String str) {
  if (SDisActive) {
    SD_buf[SD_buf_index].concat(str);
  }
}

void flash_SD() {
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
    SDisActive=false;
    SD.end();
  }
  digitalWrite(LED_BUILTIN, LOW);

  SD_buf[previous_index] = "";
  //https://www.arduino.cc/reference/en/language/variables/data-types/string/functions/reserve/
  //SD_buf[previous_index].reserve(sizeof(char) * 32768);
}

void callout_altitude() {
  //ToDo
}
