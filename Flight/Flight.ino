#include <DueTimer.h>

#define SerialAir Serial
#define SerialGPS Serial1
#define SerialTWE Serial1
#define SerialUnder Serial2
#define SerialMainSD Serial3
#define SerialICS Serial3

#include <Geometry.h>
using namespace Geometry;
using namespace BLA;
char TWE_BUF[256];

#define BUFSIZE 256
//char SD_Under[BUFSIZE];
//char SD_AirData[BUFSIZE];
//char SD_ICS[BUFSIZE];
//char SD_IMU[BUFSIZE];
//char SD_PRESSURE[BUFSIZE];
//char SD_GPS[BUFSIZE];
char UART_SD[512];

#include <TORICA_UART.h>
TORICA_UART Under_UART(&SerialUnder);
TORICA_UART Air_UART(&SerialAir);
TORICA_UART Main_UART(&SerialMainSD);
const int LED_Under = 66;
const int LED_Air = A11;

#include <TORICA_ICS.h>
TORICA_ICS ics(&SerialICS);
const int LED_ICS = 67;

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


//#include "TORICA_talk.h"
bool enable_callout = false;

float accx_history_mss[20];
const int accx_history_length = 20;
float accx_ave_mss = 0;

//長谷部が追加
float velx_history_ms[20];
const int velx_history_length = 20;
float velx_ave_ms = 0;

const float FilterCoefficient = 0.65;
float lowpass = 0;
float highpass = 0;

const float Time_interval = 0.04;
float oldAcc = 0;


float urm_altitude_history_m[3];
const int urm_altitude_history_length = 3;
float urm_altitude_ave_m = 0;

// callput value
#include "MovingAverageFloat.h"
MovingAverageFloat<10> filtered_airspeed_ms;

// filtered:移動平均
// lake:対地高度, 無印:気圧基準海抜高度
// dps:気圧高度
// urm:超音波高度

// 現在の気圧高度
MovingAverageFloat<5> filtered_main_dps_altitude_m;
MovingAverageFloat<5> filtered_under_dps_altitude_m;
MovingAverageFloat<5> filtered_air_dps_altitude_m;
// プラホの高度
MovingAverageFloat<50> main_dps_altitude_platform_m;
MovingAverageFloat<50> under_dps_altitude_platform_m;
MovingAverageFloat<50> air_dps_altitude_platform_m;

#include "QuickStats.h"
// 3つの気圧高度にそれぞれ移動平均をとってプラホを10mとし，中央値をとった値で，気圧センサを用いた信頼できる対地高度．
float dps_altitude_lake_array_m[3];
QuickStats dps_altitude_lake_m;

float estimated_altitude_lake_m = 10.0;


// ---- sensor data value  ----
//    data_マイコン名_センサー名_データ種類_単位
volatile float data_main_bno_accx_mss = 0;
volatile float data_main_bno_accy_mss = 0;
volatile float data_main_bno_accz_mss = 0;
volatile float data_main_bno_qw = 0;
volatile float data_main_bno_qx = 0;
volatile float data_main_bno_qy = 0;
volatile float data_main_bno_qz = 0;
volatile float data_main_bno_roll = 0;
volatile float data_main_bno_pitch = 0;
volatile float data_main_bno_yaw = 0;

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

// ----------------------------

void setup() {
  pinMode(LED_ICS, OUTPUT);
  pinMode(LED_Under, OUTPUT);
  pinMode(LED_Air, OUTPUT);

  //delay for program flash
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_ICS, HIGH);
    digitalWrite(LED_Under, HIGH);
    digitalWrite(LED_Air, HIGH);
    delay(400);
    digitalWrite(LED_ICS, LOW);
    digitalWrite(LED_Under, LOW);
    digitalWrite(LED_Air, LOW);
    delay(100);
  }

  SerialGPS.begin(115200);  //SerialTWE
  SerialICS.begin(115200);  //SerialMainSD
  SerialAir.begin(460800);
  SerialUnder.begin(460800);
  SerialUSB.begin(115200);
  SerialTWE.print("loading...\n\n");

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
  Wire1.begin();
  Wire1.setClock(100000);
  for (int i = 0; i < accx_history_length; i++) {
    accx_history_mss[i] = 0;
  }
  for (int i = 0; i < velx_history_length; i++) {
    velx_history_ms[i] = 0;
  }
  for (int i = 0; i < urm_altitude_history_length; i++) {
    urm_altitude_history_m[i] = 0;
  }


  //delay for sensor wake up
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_ICS, HIGH);
    digitalWrite(LED_Under, HIGH);
    digitalWrite(LED_Air, HIGH);
    delay(100);
    digitalWrite(LED_ICS, LOW);
    digitalWrite(LED_Under, LOW);
    digitalWrite(LED_Air, LOW);
    delay(400);
  }
}


void loop() {

  uint32_t ISR_now_time = millis();
  static uint32_t ISR_last_time = 0;
  if (ISR_now_time - ISR_last_time >= 10) {
    ISR_last_time = millis();
    ISR_100Hz();
  }

  calculate_altitude();

  uint32_t callout_now_time = millis();
  static uint32_t callout_last_time = 0;
  if (callout_now_time - callout_last_time >= 1000) {
    callout_last_time = callout_now_time;
    if (enable_callout) {
      //callout_altitude();
    }
  }

  static uint8_t TWE_downlink_type = 0;
  static uint32_t TWE_last_send_time = millis() - 1000;
  if (TWE_downlink_type == 0 && millis() - TWE_last_send_time >= 2000) {
    SerialTWE.print("\n\n\n");
    SerialTWE.print("MAIN\n");
    sprintf(TWE_BUF, "accX    accY    accZ\n%+06.2f  %+06.2f  %+06.2f\n", data_main_bno_accx_mss, data_main_bno_accy_mss, data_main_bno_accz_mss);
    SerialTWE.print(TWE_BUF);
    sprintf(TWE_BUF, "roll(left+)   pitch   yaw\n%+06.2f        %+06.2f  %+06.2f\n", data_main_bno_roll, data_main_bno_pitch, data_main_bno_yaw);
    SerialTWE.print(TWE_BUF);
    TWE_downlink_type++;
    TWE_last_send_time = millis();
  } else if (TWE_downlink_type == 1 && millis() - TWE_last_send_time >= 400) {
    sprintf(TWE_BUF, "pressure        temp    alt\n%+06.2f        %+06.2f  %+06.2f\n", data_main_dps_pressure_hPa, data_main_dps_temperature_deg, data_main_dps_altitude_m);
    SerialTWE.print(TWE_BUF);
    SerialTWE.print("\n");
    TWE_downlink_type++;
    TWE_last_send_time = millis();
  } else if (TWE_downlink_type == 2 && millis() - TWE_last_send_time >= 400) {
    SerialTWE.print("UNDER\n");
    sprintf(TWE_BUF, "pressure        temp    alt\n%+08.2f        %+06.2f  %+06.2f\n", data_under_dps_pressure_hPa, data_under_dps_temperature_deg, data_under_dps_altitude_m);
    SerialTWE.print(TWE_BUF);
    sprintf(TWE_BUF, "sonic_alt\n%+06.2f\n", data_under_urm_altitude_m);
    SerialTWE.print(TWE_BUF);
    SerialTWE.print("\n");
    TWE_downlink_type++;
    TWE_last_send_time = millis();
  } else if (TWE_downlink_type == 3 && millis() - TWE_last_send_time >= 400) {
    SerialTWE.print("AIR\n");
    sprintf(TWE_BUF, "pressure        temp    alt\n%+08.2f        %+06.2f  %+06.2f\n", data_air_dps_pressure_hPa, data_air_dps_temperature_deg, data_air_dps_altitude_m);
    SerialTWE.print(TWE_BUF);
    sprintf(TWE_BUF, "diffPressure    AirSpeed\n%+08.3f        %+06.2f\n", data_air_sdp_differentialPressure_Pa, data_air_sdp_airspeed_mss);
    SerialTWE.print(TWE_BUF);
    SerialTWE.print("\n");
    TWE_downlink_type++;
    TWE_last_send_time = millis();
  } else if (TWE_downlink_type == 4 && millis() - TWE_last_send_time >= 400) {
    SerialTWE.print("ICS(joystick)\n");
    sprintf(TWE_BUF, "angle (center=7500)\n%d\n", data_ics_angle);
    SerialTWE.print(TWE_BUF);
    //Reset downlink type
    TWE_downlink_type = 0;
    TWE_last_send_time = millis();
  }
}


void ISR_100Hz() {
  uint32_t time_us = micros();
  uint32_t time_ms = millis();

  polling_UART();

  //発進判定のため，測定のみ100Hz
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Quaternion quat = bno.getQuat();
  //imu::Vector<3> magnet = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);    magnet.x()
  //imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);         gyro.x()
  //imu::Vector<3> ground_acc = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL); ground_acc.x()
  data_main_bno_accx_mss = accel.x();
  data_main_bno_accy_mss = accel.y();
  data_main_bno_accz_mss = accel.z();
  //ここから どうにかする FLAG
  if (!enable_callout) {
    accx_ave_mss = 0;
    for (int i = accx_history_length - 1; i > 0; i--) {
      accx_history_mss[i] = accx_history_mss[i - 1];
      accx_ave_mss += accx_history_mss[i];
    }
    accx_history_mss[0] = (-1) * data_main_bno_accx_mss;
    accx_ave_mss += accx_history_mss[0];
    accx_ave_mss /= accx_history_length;

    velx_ave_ms = 0;
    for (int i = velx_history_length - 1; i > 0; i--) {
      velx_history_ms[i] = velx_history_ms[i - 1];
      velx_ave_ms += velx_history_ms[i];
    }

    lowpass = lowpass * FilterCoefficient + accx_ave_mss * (1 - FilterCoefficient);
    highpass = accx_ave_mss - lowpass;

    velx_history_ms[0] = ((highpass + oldAcc) * Time_interval) / 2 + velx_history_ms[1];
    oldAcc = highpass;

    velx_ave_ms += velx_history_ms[0];
    velx_ave_ms /= velx_history_length;

    if (velx_ave_ms > 0.4) {
      enable_callout = true;
    }

    SerialUSB.println(velx_ave_ms);
  }


  //ここまで

  static int loop_count = 0;
  if (loop_count == 0) {
    data_main_bno_qw = quat.w();
    data_main_bno_qx = quat.x();
    data_main_bno_qy = quat.y();
    data_main_bno_qz = quat.z();
    Quaternion qua(data_main_bno_qx, data_main_bno_qy, data_main_bno_qz, data_main_bno_qw);
    EulerAngles euler(qua.to_rotation_matrix());
    data_main_bno_roll = -(euler.first() * 180 / 3.1415);
    data_main_bno_pitch = euler.second() * 180 / 3.1415;
    data_main_bno_yaw = euler.third() * 180 / 3.1415;
    sprintf(UART_SD, "%d, %.2f,%.2f,%.2f,", time_ms,
            data_main_bno_accx_mss, data_main_bno_accy_mss, data_main_bno_accz_mss);
    /*
      sprintf(SD_IMU, "IMU,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", time_ms,
            data_main_bno_accx_mss, data_main_bno_accy_mss, data_main_bno_accz_mss,
            data_main_bno_qw,   data_main_bno_qx,   data_main_bno_qy,   data_main_bno_qz );
    */
    //SerialMainSD.print(SD_IMU);
  } else if (loop_count == 1) {
    sprintf(UART_SD, "%.2f,%.2f,%.2f,%.2f, %.2f,%.2f,%.2f,",
            data_main_bno_qw, data_main_bno_qx, data_main_bno_qy, data_main_bno_qz,
            data_main_bno_roll, data_main_bno_pitch, data_main_bno_yaw);
  } else if (loop_count == 2) {
    if (dps.temperatureAvailable() && dps.pressureAvailable()) {
      dps.getEvents(&temp_event, &pressure_event);
      data_main_dps_pressure_hPa = pressure_event.pressure;
      data_main_dps_temperature_deg = temp_event.temperature;
      data_main_dps_altitude_m = (pow(1013.25 / data_main_dps_pressure_hPa, 1 / 5.257) - 1) * (data_main_dps_temperature_deg + 273.15) / 0.0065;
      //sprintf(SD_PRESSURE, "PRESSURE,%d,%.2f,%.2f,%.2f\n", time_ms, data_main_dps_pressure_hPa, data_main_dps_temperature_deg, data_main_dps_altitude_m);
      //SerialMainSD.print(SD_PRESSURE);
      filtered_main_dps_altitude_m.add(data_main_dps_altitude_m);
      if (!enable_callout) {
        main_dps_altitude_platform_m.add(data_main_dps_altitude_m);
      }
    }
    sprintf(UART_SD, "%.2f,%.2f,%.2f, %.2f,%.2f,%.2f, %.2f,",
            data_main_dps_pressure_hPa, data_main_dps_temperature_deg, data_main_dps_altitude_m,
            data_under_dps_pressure_hPa, data_under_dps_temperature_deg, data_under_dps_altitude_m, data_under_urm_altitude_m);
    /*if (!enable_callout) {
      urm_altitude_ave_m = 0;
      for (int i = urm_altitude_history_length - 1; i > 0; i--) {
        urm_altitude_history_m[i] = urm_altitude_history_m[i - 1];
        urm_altitude_ave_m += urm_altitude_history_m[i];
      }
      urm_altitude_history_m[0] = data_under_urm_altitude_m;
      urm_altitude_ave_m += urm_altitude_history_m[0];
      urm_altitude_ave_m /= urm_altitude_history_length;

      if (urm_altitude_ave_m > 9.0) {
        enable_callout = true;
      }
    }*/
  } else if (loop_count == 3) {
    sprintf(UART_SD, "%.2f,%.2f,%.2f, %.2f,%.2f, %d,",
            data_air_dps_pressure_hPa, data_air_dps_temperature_deg, data_air_dps_altitude_m,
            data_air_sdp_differentialPressure_Pa, data_air_sdp_airspeed_mss,
            data_ics_angle);
  } else {
    sprintf(UART_SD, "%u,%u,%u.%u,%10.7lf,%10.7lf,%5.2lf\n",
            data_main_gps_hour, data_main_gps_minute, data_main_gps_second, data_main_gps_centisecond,
            data_main_gps_latitude_deg, data_main_gps_longitude_deg, data_main_gps_altitude_m);
    loop_count = -1;
  }
  loop_count++;
  //バッファをクリアしてから新しいデータを書き込み
  SerialMainSD.flush();
  SerialAir.flush();
  SerialUnder.flush();
  SerialMainSD.print(UART_SD);
  SerialAir.print(UART_SD);
  SerialUnder.print(UART_SD);

  if (micros() - time_us > 9900) {  //MAX10000=100Hz
    SerialUSB.print("ISR100Hz_overrun!!!");
  }
  SerialUSB.print("ISR_us:");
  SerialUSB.println(micros() - time_us);
}


void polling_UART() {
  //ICS
  data_ics_angle = ics.read_Angle();
  if (data_ics_angle > 0) {
    digitalWrite(LED_ICS, !digitalRead(LED_ICS));
    //sprintf(SD_ICS, "RUDDER,%d,%d\n", time_ms, data_ics_angle);
    //SerialMainSD.print(SD_ICS);
    //digitalWrite(LED_ICS, LOW);
  }

  //UnderSide
  int readnum = Under_UART.readUART();
  if (readnum == 4) {
    digitalWrite(LED_Under, !digitalRead(LED_Under));
    data_under_dps_pressure_hPa = Under_UART.UART_data[0];
    data_under_dps_temperature_deg = Under_UART.UART_data[1];
    data_under_dps_altitude_m = Under_UART.UART_data[2];
    data_under_urm_altitude_m = Under_UART.UART_data[3];
    //sprintf(SD_Under, "UNDER,%d,%.2f,%.2f,%.2f,%.2f\n", time_ms, data_under_dps_pressure_hPa, data_under_dps_temperature_deg, data_under_dps_altitude_m, data_under_urm_altitude_m );
    //SerialMainSD.print(SD_Under);
    //digitalWrite(LED_Under, LOW);
    filtered_under_dps_altitude_m.add(data_under_dps_altitude_m);
    if (!enable_callout) {
      under_dps_altitude_platform_m.add(data_under_dps_altitude_m);
    }
  }

  //AirData
  readnum = Air_UART.readUART();
  if (readnum == 5) {
    digitalWrite(LED_Air, !digitalRead(LED_Air));
    data_air_dps_pressure_hPa = Air_UART.UART_data[0];
    data_air_dps_temperature_deg = Air_UART.UART_data[1];
    data_air_dps_altitude_m = Air_UART.UART_data[2];
    data_air_sdp_differentialPressure_Pa = Air_UART.UART_data[3];
    data_air_sdp_airspeed_mss = Air_UART.UART_data[4];
    filtered_airspeed_ms.add(data_air_sdp_airspeed_mss);
    //sprintf(SD_AirData, "AIR,%d,%.2f,%.2f,%.2f,%.2f,%.2f\n", time_ms, data_air_dps_pressure_hPa, data_air_dps_temperature_deg, data_air_dps_altitude_m, data_air_sdp_differentialPressure_Pa, data_air_sdp_airspeed_mss );
    //SerialMainSD.print(SD_AirData);
    //digitalWrite(LED_Air, LOW);
    filtered_air_dps_altitude_m.add(data_air_dps_altitude_m);
    if (!enable_callout) {
      air_dps_altitude_platform_m.add(data_air_dps_altitude_m);
    }
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
      /*
        sprintf(SD_GPS, "GPS,%d,%d,%d,%d,%d,%.6lf,%.6lf,%.2lf\n", time_ms,
              data_main_gps_hour,         data_main_gps_minute,        data_main_gps_second,    data_main_gps_centisecond,
              data_main_gps_latitude_deg, data_main_gps_longitude_deg, data_main_gps_altitude_m );*/
      //SerialMainSD.print(SD_GPS);
    }
  }
}


void calculate_altitude() {
  dps_altitude_lake_array_m[0] = filtered_main_dps_altitude_m.get() - main_dps_altitude_platform_m.get() + 10.0;
  dps_altitude_lake_array_m[1] = filtered_under_dps_altitude_m.get() - under_dps_altitude_platform_m.get() + 10.0;
  dps_altitude_lake_array_m[2] = filtered_air_dps_altitude_m.get() - air_dps_altitude_platform_m.get() + 10.0;

  dps_altitude_lake_m.median(dps_altitude_lake_array_m, 3);
}


void callout_altitude() {
  //ToDo
  Wire1.beginTransmission(0x2E);  // スタートとスレーブアドレスを送る役割　（swの役割）

  char str[] = "teikuohu";
  Wire1.write(str, strlen(str) * sizeof(char));
  Wire1.write('\r');

  Wire1.endTransmission();  // stop transmitting

  static int step_altitude_lake_m = 10;
  if (estimated_altitude_lake_m <= step_altitude_lake_m - 1) {
    step_altitude_lake_m--;
    //callout_altitude(step_altitude_lake_m);
  } else {
    //callout_airspeed(filtered_airspeed_ms.get());
  }
}
