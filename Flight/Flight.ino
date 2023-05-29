#include <SdFat.h>
#include <DueTimer.h>

#define SerialICS   Serial
#define SerialGPS   Serial1
#define SerialTWE   Serial1
#define SerialAir   Serial2
#define SerialUnder Serial3

#include <Geometry.h>
using namespace Geometry;
using namespace BLA;
char TWE_BUF[256];

//SD系の初期設定

const uint8_t SD_CS_PIN = 28;
#define SPI_CLOCK SD_SCK_MHZ(10)

//書き込むデータの設定
const size_t BUF_SIZE = 256;
char SD_BUF[BUF_SIZE];

//file size(byte)
const uint32_t FILE_SIZE = 1048576;

#define SD_CONFIG SdSpiConfig(SD_CS_PIN,DEDICATED_SPI,SPI_CLOCK)

/*
  #include <TORICA_SD.h>
  TORICA_SD main_SD(cs_SD);
*/
const int cs_SD = A8;
const int LED_SD = A11;

//インスタンス化
SdFat main_sd;
File file;

char fileName[16];


#define BUFSIZE 256
char SD_Under[BUFSIZE];
char SD_AirData[BUFSIZE];
char SD_ICS[BUFSIZE];
char SD_IMU[BUFSIZE];
char SD_PRESSURE[BUFSIZE];
char SD_GPS[BUFSIZE];
char UART_SD[512];

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

// ---- sensor data value  ----
//    data_マイコン名_センサー名_データ種類_単位
volatile float data_main_bno_accx_mss = 0;
volatile float data_main_bno_accy_mss = 0;
volatile float data_main_bno_accz_mss = 0;
volatile float data_main_bno_qw = 0;
volatile float data_main_bno_qx = 0;
volatile float data_main_bno_qy = 0;
volatile float data_main_bno_qz = 0;

volatile float data_main_dps_pressure_hPa = 0;
volatile float data_main_dps_temperature_deg = 0;
volatile float data_main_dps_altitude_m = 0;

volatile float data_under_dps_pressure_hPa = 0;
volatile float data_under_dps_temperature_deg = 0;
volatile float data_under_dps_altitude_m = 0;
volatile float data_under_urm_altitude_m = 0;

volatile float data_air_dps_pressure_hPa = 0;
volatile float data_air_dps_temperature_deg = 0;
volatile float data_air_dps_altitude_m = 0;
volatile float data_air_sdp_differentialPressure_Pa = 0;
volatile float data_air_sdp_airspeed_mss = 0;

volatile int data_ics_angle = 0;

volatile uint8_t data_main_gps_hour = 0;
volatile uint8_t data_main_gps_minute = 0;
volatile uint8_t data_main_gps_second = 0;
volatile uint8_t data_main_gps_centisecond = 0;
volatile double data_main_gps_latitude_deg = 0;
volatile double data_main_gps_longitude_deg = 0;
volatile double data_main_gps_altitude_m = 0;

volatile bool SDisActive = false;



// ----------------------------

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
  SerialTWE.print("loading...\n\n");

  SDisActive = main_sd.begin(SD_CONFIG);

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

  //delay for sensor wake up
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

  //NVIC_SetPriority((IRQn_Type)SysTick_IRQn, 14);
  //NVIC_SetPriority((IRQn_Type)TC3_IRQn, 15);  //https://github.com/ivanseidel/DueTimer/blob/master/DueTimer.cpp#L17
  //NVIC_SetPriority((IRQn_Type)TC4_IRQn, 15);  //https://github.com/ivanseidel/DueTimer/blob/master/DueTimer.cpp#L18
  //Timer3.attachInterrupt(ISR_100Hz).start(10000);
  //Timer4.attachInterrupt(ISR_200Hz).start(5000);
}

void loop() {
  if (SDisActive) {
    digitalWrite(LED_SD, !digitalRead(LED_SD));
  } else {
    digitalWrite(LED_SD, LOW);
  }
  //main_SD.flash();

  uint32_t callout_now_time = millis();
  static uint32_t callout_last_time = 0;
  /*if (callout_now_time - callout_last_time >= 1000) {
    callout_last_time = callout_now_time;
    callout_altitude();
    }*/

  uint32_t TWE_now_time = millis();
  static uint32_t TWE_last_time = 0;
  static uint8_t TWE_downlink_type = 0;
  if (TWE_downlink_type == 0 && millis() > 4000) {
    TWE_downlink_type++;
    SerialTWE.print("MAIN\n");
    sprintf(TWE_BUF, "accX    accY    accZ\n%+06.2f  %+06.2f  %+06.2f\n", data_main_bno_accx_mss, data_main_bno_accy_mss, data_main_bno_accz_mss);
    SerialTWE.print(TWE_BUF);
    sprintf(TWE_BUF, "qW      qX      qY      qZ\n%+06.2f  %+06.2f  %+06.2f  %+06.2f\n", data_main_bno_qw, data_main_bno_qx, data_main_bno_qy, data_main_bno_qz);
    SerialTWE.print(TWE_BUF);
  } else if (TWE_downlink_type == 1 && millis() > 4200) {
    TWE_downlink_type++;
    sprintf(TWE_BUF, "pressure        temp    alt\n%+06.2f        %+06.2f  %+06.2f\n", data_main_dps_pressure_hPa, data_main_dps_temperature_deg, data_main_dps_altitude_m);
    SerialTWE.print(TWE_BUF);
    SerialTWE.print("\n");
  } else if (TWE_downlink_type == 2 && millis() > 4400) {
    TWE_downlink_type++;
    SerialTWE.print("UNDER\n");
    sprintf(TWE_BUF, "pressure        temp    alt\n%+08.2f        %+06.2f  %+06.2f\n", data_under_dps_pressure_hPa, data_under_dps_temperature_deg, data_under_dps_altitude_m);
    SerialTWE.print(TWE_BUF);
    sprintf(TWE_BUF, "sonic_alt\n%+06.2f\n", data_under_urm_altitude_m);
    SerialTWE.print(TWE_BUF);
    SerialTWE.print("\n");
  } else if (TWE_downlink_type == 3 && millis() > 4800) {
    TWE_downlink_type++;
    SerialTWE.print("AIR\n");
    sprintf(TWE_BUF, "pressure        temp    alt\n%+08.2f        %+06.2f  %+06.2f\n", data_air_dps_pressure_hPa, data_air_dps_temperature_deg, data_air_dps_altitude_m);
    SerialTWE.print(TWE_BUF);
    sprintf(TWE_BUF, "diffPressure    AirSpeed\n%+08.3f        %+06.2f\n", data_air_sdp_differentialPressure_Pa,  data_air_sdp_airspeed_mss);
    SerialTWE.print(TWE_BUF);
    SerialTWE.print("\n");
  } else if (TWE_downlink_type == 4 && millis() > 5000) {
    TWE_downlink_type++;
    SerialTWE.print("ICS(joystick)\n");
    sprintf(TWE_BUF, "angle (center=7500)\n%d\n", data_ics_angle);
    SerialTWE.print(TWE_BUF);
    SerialTWE.print("\n\n\n");
  } else if (TWE_downlink_type == 5) {
    if (TWE_now_time - TWE_last_time >= 250) {
      TWE_last_time = TWE_now_time;
      TWE_downlink();
    }
  }


  //元割り込み
  if (SDisActive)
  {

    String s;
    int fileNum = 0;

    while (1)
    {
      s = "LOG";
      if (fileNum < 10)
      {
        s += "000";
      }
      else if (fileNum < 100)
      {
        s += "00";
      }
      else if (fileNum < 1000)
      {
        s += "0";
      }
      s += fileNum;
      s += ".CSV";
      s.toCharArray(fileName, 16);
      if (!main_sd.exists(fileName))
        break;
      fileNum++;
    }


    static uint32_t time = micros();
    static uint32_t time_ms = millis();

    if (!file.open(fileName, O_RDWR | O_CREAT | O_TRUNC)) {
      Serial.println("SD card initialization failed.");
    }
    else {
      Serial.println("SD card initialization succeeded.");
    }

    //ICS
    data_ics_angle = ics.read_Angle();
    if (data_ics_angle > 0) {
      digitalWrite(LED_ICS, HIGH);
      sprintf(SD_ICS, "RUDDER,%d,%d\n", time_ms, data_ics_angle);
      file.print(SD_ICS);
      digitalWrite(LED_ICS, LOW);
    }

    //UnderSide
    int readnum = Under_UART.readUART();
    if (readnum == 4) {
      digitalWrite(LED_Under, HIGH);
      data_under_dps_pressure_hPa = Under_UART.UART_data[0];
      data_under_dps_temperature_deg = Under_UART.UART_data[1];
      data_under_dps_altitude_m = Under_UART.UART_data[2];
      data_under_urm_altitude_m = Under_UART.UART_data[3];
      sprintf(SD_Under, "UNDER,%d,%.2f,%.2f,%.2f,%.2f\n", time_ms, data_under_dps_pressure_hPa, data_under_dps_temperature_deg, data_under_dps_altitude_m, data_under_urm_altitude_m );
      file.print(SD_Under);
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
      sprintf(SD_AirData, "AIR,%d,%.2f,%.2f,%.2f,%.2f,%.2f\n", time_ms, data_air_dps_pressure_hPa, data_air_dps_temperature_deg, data_air_dps_altitude_m, data_air_sdp_differentialPressure_Pa, data_air_sdp_airspeed_mss );
      file.print(SD_AirData);
      digitalWrite(LED_Air, LOW);
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
        sprintf(SD_GPS, "GPS,%d,%d,%d,%d,%d,%.6lf,%.6lf,%.2lf\n", time_ms,
                data_main_gps_hour,         data_main_gps_minute,        data_main_gps_second,    data_main_gps_centisecond,
                data_main_gps_latitude_deg, data_main_gps_longitude_deg, data_main_gps_altitude_m );
        file.print(SD_GPS);
      }
    }

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
    sprintf(SD_IMU, "IMU,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", time_ms,
            data_main_bno_accx_mss, data_main_bno_accy_mss, data_main_bno_accz_mss,
            data_main_bno_qw,   data_main_bno_qx,   data_main_bno_qy,   data_main_bno_qz );
    file.print(SD_IMU);

    if (dps.temperatureAvailable() && dps.pressureAvailable()) {
      dps.getEvents(&temp_event, &pressure_event);
      data_main_dps_pressure_hPa = pressure_event.pressure;
      data_main_dps_temperature_deg = temp_event.temperature;
      data_main_dps_altitude_m = (pow(1013.25 / data_main_dps_pressure_hPa, 1 / 5.257) - 1) * (data_main_dps_temperature_deg + 273.15) / 0.0065;
      sprintf(SD_PRESSURE, "PRESSURE,%d,%.2f,%.2f,%.2f\n", time_ms, data_main_dps_pressure_hPa, data_main_dps_temperature_deg, data_main_dps_altitude_m);
      file.print(SD_PRESSURE);
    }

    static int loop_count_sd = 0;
    if (loop_count_sd == 0) {
      sprintf(UART_SD, "%d, %.2f,%.2f,%.2f, %.2f,%.2f,%.2f,%.2f,", time_ms,
              data_main_bno_accx_mss, data_main_bno_accy_mss, data_main_bno_accz_mss,
              data_main_bno_qw,   data_main_bno_qx,   data_main_bno_qy,   data_main_bno_qz
             );
    }
    else if (loop_count_sd == 1) {
      sprintf(UART_SD, "%.2f,%.2f,%.2f, %.2f,%.2f,%.2f, %.2f,",
              data_main_dps_pressure_hPa, data_main_dps_temperature_deg, data_main_dps_altitude_m,
              data_under_dps_pressure_hPa, data_under_dps_temperature_deg, data_under_dps_altitude_m, data_under_urm_altitude_m
             );
    }
    else if (loop_count_sd == 2) {
      sprintf(UART_SD, "%.2f,%.2f,%.2f, %.2f,%.2f, %d,",
              data_air_dps_pressure_hPa,  data_air_dps_temperature_deg,  data_air_dps_altitude_m,
              data_air_sdp_differentialPressure_Pa,  data_air_sdp_airspeed_mss,
              data_ics_angle
             );
    }
    else {
      sprintf(UART_SD, "%u,%u,%u.%u,%10.7lf,%10.7lf,%5.2lf\n",
              data_main_gps_hour,  data_main_gps_minute,  data_main_gps_second, data_main_gps_centisecond,
              data_main_gps_latitude_deg,  data_main_gps_longitude_deg, data_main_gps_altitude_m
             );
      loop_count_sd = -1;
    }
    SerialAir.print(UART_SD);
    delayMicroseconds(500);
    SerialUnder.print(UART_SD);
    loop_count_sd++;


    if (micros() - time > 9900) {  //MAX10000=100Hz
      SerialUSB.print("ISR100Hz_overrun!!!");
    }
    SerialUSB.print("ISR_us:");
    SerialUSB.println(micros() - time);

    file.close();
  }

}

void TWE_downlink() {
  Quaternion qua(data_main_bno_qw, data_main_bno_qy, data_main_bno_qz, data_main_bno_qw);
  EulerAngles euler(qua.to_rotation_matrix());

  SerialTWE.print("+roll means left wing up\n");
  //delay(10);
  SerialTWE.print("pitch[deg] roll[deg] IAS[m/s]\n");
  //delay(10);
  sprintf(TWE_BUF, "%+06.2f     %+06.2f    %.2f\n", euler.second() * 180 / 3.1415 , -(euler.first() * 180 / 3.1415) , data_air_sdp_airspeed_mss);
  SerialTWE.print(TWE_BUF);
  SerialTWE.print("\n\n");
}

void callout_altitude() {
  //ToDo
}
