#include <DueTimer.h>

#define SerialICS   Serial
#define SerialGPS   Serial1
#define SerialTWE   Serial1
#define SerialAir   Serial2
#define SerialUnder Serial3

#include <TORICA_SD.h>
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
char UART_SD[BUFSIZE];

#include <TORICA_UART.h>
TORICA_UART Under_UART(&SerialUnder);
TORICA_UART Air_UART(&SerialAir);
const int LED_Under = 12;
const int LED_Air = A0;

#include <TORICA_ICS.h>
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


// ---- sensor data value  ----
//    data_マイコン名_センサー名_データ種類_単位
float data_main_bno_accx_mss = 0;
float data_main_bno_accy_mss = 0;
float data_main_bno_accz_mss = 0;
float data_main_bno_qw = 0;
float data_main_bno_qx = 0;
float data_main_bno_qy = 0;
float data_main_bno_qz = 0;

float data_main_dps_pressure_hPa = 0;
float data_main_dps_temperature_deg = 0;
float data_main_dps_altitude_m = 0;

float data_under_dps_pressure_hPa = 0;
float data_under_dps_temperature_deg = 0;
float data_under_dps_altitude_m = 0;
float data_under_urm_altitude_m = 0;

float data_air_dps_pressure_hPa = 0;
float data_air_dps_temperature_deg = 0;
float data_air_dps_altitude_m = 0;
float data_air_sdp_differentialPressure_Pa = 0;
float data_air_sdp_airspeed_mss = 0;

int data_ics_angle = 0;

uint8_t data_main_gps_hour = 0;
uint8_t data_main_gps_minute = 0;
uint8_t data_main_gps_second = 0;
uint8_t data_main_gps_centisecond = 0; 
double data_main_gps_latitude_deg = 0;
double data_main_gps_longitude_deg = 0;
double data_main_gps_altitude_m = 0;

// ----------------------------

void ISR_readUART_500Hz() {
  //  unsigned long time = micros();

  //UnderSide
  int readnum = Under_UART.readUART();
  if (readnum == 4) {
    digitalWrite(LED_Under, HIGH);
    data_under_dps_pressure_hPa = Under_UART.UART_data[0];
    data_under_dps_temperature_deg = Under_UART.UART_data[1];
    data_under_dps_altitude_m = Under_UART.UART_data[2];
    data_under_urm_altitude_m = Under_UART.UART_data[3];
    sprintf(SD_Under, "UNDER,%d,%.2f,%.2f,%.2f,%.2f\n", millis(), data_under_dps_pressure_hPa, data_under_dps_temperature_deg, data_under_dps_altitude_m, data_under_urm_altitude_m );
    main_SD.add_str(SD_Under);
    digitalWrite(LED_Under, LOW);
  }

  //AirData
  readnum = Air_UART.readUART();
  if (readnum == 5) {
    digitalWrite(LED_Air, HIGH);
    data_air_dps_pressure_hPa = Air_UART.UART_data[0];
    data_air_dps_temperature_deg = Air_UART.UART_data[1];
    data_air_dps_altitude_m = Air_UART.UART_data[2];
    data_air_sdp_differentialPressure_Pa = Air_UART.UART_data[3];
    data_air_sdp_airspeed_mss = Air_UART.UART_data[4];
    sprintf(SD_AirData, "AIR,%d,%.2f,%.2f,%.2f,%.2f,%.2f\n", millis(), data_air_dps_pressure_hPa, data_air_dps_temperature_deg, data_air_dps_altitude_m, data_air_sdp_differentialPressure_Pa, data_air_sdp_airspeed_mss );
    main_SD.add_str(SD_AirData);
    digitalWrite(LED_Air, LOW);
  }

  //ICS
  data_ics_angle = ics.read_Angle();
  if (data_ics_angle > 0) {
    digitalWrite(LED_ICS, HIGH);
    sprintf(SD_ICS, "RUDDER,%d,%d\n", millis(), data_ics_angle);
    main_SD.add_str(SD_ICS);
    digitalWrite(LED_ICS, LOW);
  }

  //GPS
  while (SerialGPS.available() > 0) {
    if (gps.encode(SerialGPS.read())) {
      data_main_gps_hour = gps.time.hour();
      data_main_gps_minute = gps.time.minute();
      data_main_gps_second = gps.time.second();
      data_main_gps_centisecond = gps.time.centisecond();
      data_main_gps_latitude_deg = gps.location.lat();
      data_main_gps_longitude_deg = gps.location.lng();
      data_main_gps_altitude_m = gps.altitude.meters();
      sprintf(SD_GPS, "GPS,%d,%d,%d,%d,%d,%.6lf,%.6lf,%.2lf\n", millis(),
              data_main_gps_hour,         data_main_gps_minute,        data_main_gps_second,    data_main_gps_centisecond,
              data_main_gps_latitude_deg, data_main_gps_longitude_deg, data_main_gps_altitude_m );
      main_SD.add_str(SD_GPS);
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
  Quaternion qua(data_main_bno_qw, data_main_bno_qy, data_main_bno_qz, data_main_bno_qw);
  EulerAngles euler(qua.to_rotation_matrix());

  Serial1.print("+roll means left wing up\n");
  delay(10);
  Serial1.print("pitch[deg] roll[deg] IAS[m/s]\n");
  delay(10);
  sprintf(TWE_BUF, "%+06.2f     %+06.2f    %.2f\n", euler.second() * 180 / 3.1415 , -(euler.first() * 180 / 3.1415) , 0.0 );
  Serial1.print(TWE_BUF);
  Serial1.print("\n");
  delay(10);
  Serial1.print("\n");
  delay(10);
}

void ISR_readI2C0_sendUART_100Hz() {
  unsigned long time = micros();
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Quaternion quat = bno.getQuat();
  //imu::Vector<3> magnet = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);    magnet.x()
  //imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);         gyro.x()
  //imu::Vector<3> ground_acc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL); ground_acc.x()
  data_main_bno_accx_mss = accel.x();
  data_main_bno_accy_mss = accel.y();
  data_main_bno_accz_mss = accel.z();
  data_main_bno_qw = quat.w();
  data_main_bno_qx = quat.x();
  data_main_bno_qy = quat.y();
  data_main_bno_qz = quat.z();
  sprintf(SD_IMU, "IMU,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", millis(),
          data_main_bno_accx_mss, data_main_bno_accy_mss, data_main_bno_accz_mss,
          data_main_bno_qw,   data_main_bno_qx,   data_main_bno_qy,   data_main_bno_qz );
  main_SD.add_str(SD_IMU);

  if (dps.temperatureAvailable() && dps.pressureAvailable()) {
    dps.getEvents(&temp_event, &pressure_event);
    data_main_dps_pressure_hPa = pressure_event.pressure;
    data_main_dps_temperature_deg = pressure_event.temperature;
    data_main_dps_altitude_m = (pow(1013.25 / data_main_dps_pressure_hPa, 1 / 5.257) - 1) * (data_main_dps_temperature_deg + 273.15) / 0.0065;
    sprintf(SD_PRESSURE, "PRESSURE,%d,%.2f,%.2f,%.2f\n", millis(), data_main_dps_pressure_hPa, data_main_dps_temperature_deg, data_main_dps_altitude_m);
    main_SD.add_str(SD_PRESSURE);
  }

  sprintf(UART_SD, "%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", millis(),
          data_main_bno_accx_mss, data_main_bno_accy_mss, data_main_bno_accz_mss,
          data_main_bno_qw,   data_main_bno_qx,   data_main_bno_qy,   data_main_bno_qz,
          data_main_dps_pressure_hPa, data_main_dps_temperature_deg, data_main_dps_altitude_m);
  SerialAir.print(UART_SD);
  SerialUnder.print(UART_SD);

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

  // data label
  sprintf(UART_SD, "time_ms,data_main_bno_accx_mss, data_main_bno_accy_mss, data_main_bno_accz_mss, data_main_bno_qw,   data_main_bno_qx,   data_main_bno_qy,   data_main_bno_qz,data_main_dps_pressure_hPa, data_main_dps_temperature_deg, data_main_dps_altitude_m\n");
  SerialAir.print(UART_SD);
  SerialUnder.print(UART_SD);

  NVIC_SetPriority((IRQn_Type)SysTick_IRQn, 13);
  NVIC_SetPriority((IRQn_Type)TC3_IRQn, 14);  //https://github.com/ivanseidel/DueTimer/blob/master/DueTimer.cpp#L17
  NVIC_SetPriority((IRQn_Type)TC4_IRQn, 15);  //https://github.com/ivanseidel/DueTimer/blob/master/DueTimer.cpp#L18
  NVIC_SetPriority((IRQn_Type)TC5_IRQn, 15);
  Timer3.attachInterrupt(ISR_readI2C0_sendUART_100Hz).start(10000);
  Timer4.attachInterrupt(ISR_readUART_500Hz).start(2000);
  Timer5.attachInterrupt(ISR_TWE_1_Hz).start(1000000);
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
