/*
  エアデータと機体下にデータ送信
  音声で伝えるデータの仕様決め
*/
#include <DueTimer.h>

#define SerialICS   Serial
#define SerialGPS   Serial1
#define SerialTWE   Serial1
#define SerialAir   Serial2
#define SerialUnder Serial3

#include "TORICA_SD.h"
const int cs_SD = A8;
TORICA_SD main_SD(cs_SD);
const int LED_SD = A11;

#define BUFSIZE 256
char SD_Under[BUFSIZE];
char SD_AirData[BUFSIZE];
char SD_ICS[BUFSIZE];
char SD_IMU[BUFSIZE];
char SD_PRESSURE[BUFSIZE];
char SD_GPS[BUFSIZE];

#include "TORICA_UART.h"
TORICA_UART Under_UART(&SerialUnder);
TORICA_UART Air_UART(&SerialAir);
const int LED_Under = 12;
const int LED_Air = A0;

#include "TORICA_ICS.h"
TORICA_ICS ics(&SerialICS);
const int LED_ICS = 13;

#include <TinyGPSPlus.h>
TinyGPSPlus gps;

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);

#include <Adafruit_DPS310.h>
Adafruit_DPS310 dps;
sensors_event_t temp_event, pressure_event;

bool timer_TBD_Hz = false;

float qw = 0;
float qx = 0;
float qy = 0;
float qz = 0;
float alt_ultrasonic_m = 0;

void log_SD(char data[BUFSIZE]) {
  main_SD.add_str(data);
  SerialAir.print(data);
  //SerialAir.flush();
  SerialUnder.print(data);
  //SerialUnder.flush();
}

void ISR_UART_500Hz() {
  //  unsigned long time = micros();

  //UnderSide
  int readnum = Under_UART.readUART();
  if (readnum == 4) {
    digitalWrite(LED_Under, HIGH);
    sprintf(SD_Under, "UNDER,%d,%.2f,%.2f,%.2f,%.2f\n", millis(), Under_UART.UART_data[0], Under_UART.UART_data[1], Under_UART.UART_data[2], Under_UART.UART_data[3] );
    log_SD(SD_Under);
    digitalWrite(LED_Under, LOW);
  }

  //AirData
  readnum = Air_UART.readUART();
  if (readnum == 5) {
    digitalWrite(LED_Air, HIGH);
    sprintf(SD_AirData, "AIR,%d,%.2f,%.2f,%.2f,%.2f,%.2f\n", millis(), Air_UART.UART_data[0], Air_UART.UART_data[1], Air_UART.UART_data[2], Air_UART.UART_data[3], Air_UART.UART_data[4] );
    log_SD(SD_AirData);
    digitalWrite(LED_Air, LOW);
  }

  //ICS
  int ics_angle = ics.read_Angle();
  if (ics_angle > 0) {
    digitalWrite(LED_ICS, HIGH);
    sprintf(SD_ICS, "RUDDER,%d,%d\n", millis(), ics_angle );
    log_SD(SD_ICS);
    digitalWrite(LED_ICS, LOW);
  }

  //GPS
  while (SerialGPS.available() > 0) {
    if (gps.encode(SerialGPS.read())) {
      sprintf(SD_GPS, "GPS,%d,%d,%d,%d,%d,%.6f,%.6f,%.2f\n", millis(), gps.time.hour(), gps.time.minute(), gps.time.second(), gps.time.centisecond(), gps.location.lat(), gps.location.lng(), gps.altitude.meters() );
      log_SD(SD_GPS);
    }
  }

  //DEBUG
  /*
    if (micros() - time > 1900) {  //MAX2000=500Hz
    SerialUSB.print("ISR500Hz_overrun!!!");
    }
    SerialUSB.print("ISR_us:");
    SerialUSB.println(micros() - time);
  */
}

#include <Geometry.h>
using namespace Geometry;
using namespace BLA;
char TWE_BUF[256];
void ISR_TWE_1_Hz() {
  Quaternion qua(qx, qy, qz, qw);
  EulerAngles euler(qua.to_rotation_matrix());
  sprintf(TWE_BUF, "pitch[deg] roll[deg] alt[m]\n%+06.2f     %+06.2f    %.2f\n\n\n", euler.second() * 180 / 3.1415, euler.first() * 180 / 3.1415, alt_ultrasonic_m );
  SerialTWE.print(TWE_BUF);
}

void ISR_I2C0_100Hz() {
  unsigned long time = micros();
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Quaternion quat = bno.getQuat();
  //imu::Vector<3> magnet = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);    magnet.x()
  //imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);         gyro.x()
  //imu::Vector<3> ground_acc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL); ground_acc.x()
  sprintf(SD_IMU, "IMU,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", millis(), accel.x(), accel.y(), accel.z(), quat.w(), quat.x(), quat.y(), quat.z() );
  qw = quat.w();
  qx = quat.x();
  qy = quat.y();
  qz = quat.z();
  log_SD(SD_IMU);

  if (dps.temperatureAvailable() && dps.pressureAvailable()) {
    dps.getEvents(&temp_event, &pressure_event);
    sprintf(SD_PRESSURE, "PRESSURE,%d,%.2f,%.2f\n", millis(), pressure_event.pressure, temp_event.temperature);
    log_SD(SD_PRESSURE);
  }

  /*
    if (micros() - time > 9900) {  //MAX10000=100Hz
    SerialUSB.print("ISR100Hz_overrun!!!");
    }
    SerialUSB.print("ISR_us:");
    SerialUSB.println(micros() - time);
  */
}

void setup() {
  pinMode(LED_ICS, OUTPUT);
  pinMode(LED_Under, OUTPUT);
  pinMode(LED_Air, OUTPUT);
  pinMode(LED_SD, OUTPUT);

  //delay for program flash
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_ICS, HIGH);
    digitalWrite(LED_Under, HIGH);
    digitalWrite(LED_Air, HIGH);
    digitalWrite(LED_SD, HIGH);
    delay(400);
    digitalWrite(LED_ICS, LOW);
    digitalWrite(LED_Under, LOW);
    digitalWrite(LED_Air, LOW);
    digitalWrite(LED_SD, LOW);
    delay(100);
  }

  SerialGPS.begin(115200);
  SerialICS.begin(115200);
  SerialAir.begin(460800);
  SerialUnder.begin(460800);
  SerialUSB.begin(115200);

  main_SD.begin();

  Wire.setClock(400000);
  if (!dps.begin_I2C()) {  // Can pass in I2C address here
    //if (! dps.begin_SPI(DPS310_CS)) {  // If you want to use SPI
    SerialUSB.println("Failed to find DPS");
    while (1) {
      digitalWrite(LED_Air, HIGH);
      delay(100);
      digitalWrite(LED_Air, LOW);
      delay(100);
    }
  }
  dps.configurePressure(DPS310_32HZ, DPS310_16SAMPLES);
  dps.configureTemperature(DPS310_32HZ, DPS310_2SAMPLES);
  SerialUSB.println("DPS OK!");

  if (!bno.begin()) {
    SerialUSB.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1) {
      digitalWrite(LED_ICS, HIGH);
      delay(100);
      digitalWrite(LED_ICS, LOW);
      delay(100);
    }
  }

  //for CALLOUT
  Wire1.setClock(400000);
  //ToDo


  //delay for sensor
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_ICS, HIGH);
    digitalWrite(LED_Under, HIGH);
    digitalWrite(LED_Air, HIGH);
    digitalWrite(LED_SD, HIGH);
    delay(100);
    digitalWrite(LED_ICS, LOW);
    digitalWrite(LED_Under, LOW);
    digitalWrite(LED_Air, LOW);
    digitalWrite(LED_SD, LOW);
    delay(400);
  }

  Timer3.attachInterrupt(ISR_I2C0_100Hz).start(10000);
  Timer4.attachInterrupt(ISR_UART_500Hz).start(2000);
  Timer5.attachInterrupt(ISR_TWE_1_Hz).start(1000000);
  NVIC_SetPriority((IRQn_Type)SysTick_IRQn, 13);
  NVIC_SetPriority((IRQn_Type)TC3_IRQn, 14);  //https://github.com/ivanseidel/DueTimer/blob/master/DueTimer.cpp#L17
  NVIC_SetPriority((IRQn_Type)TC4_IRQn, 15);  //https://github.com/ivanseidel/DueTimer/blob/master/DueTimer.cpp#L18
}

void loop() {
  if (main_SD.SDisActive) {
    digitalWrite(LED_SD, HIGH);
  }
  main_SD.flash();
  digitalWrite(LED_SD, LOW);

  delay(10);

  if (timer_TBD_Hz) {
    timer_TBD_Hz = false;
    callout_altitude();
  }
}

void callout_altitude() {
  //ToDo
}
