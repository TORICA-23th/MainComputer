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

#include <TORICA_talk.h>
TORICA_talk speaker;

enum {
  PLATFORM,
  TAKEOFF,
  HIGH_LEVEL,
  MID_LEVEL,
  LOW_LEVEL
} flight_phase = PLATFORM;


// filtered:移動平均
// lake:対地高度, 無印:気圧基準海抜高度
// dps:気圧高度
// urm:超音波高度

const float const_platform_m = 10.6;

#include "TORICA_MoveAve.h"
// 対気速度
TORICA_MoveAve<5> filtered_airspeed_ms(10.2);

// 現在の気圧高度(気圧基準)
TORICA_MoveAve<5> filtered_main_dps_altitude_m(0);
TORICA_MoveAve<5> filtered_under_dps_altitude_m(0);
TORICA_MoveAve<5> filtered_air_dps_altitude_m(0);
// プラホの高度(気圧基準)
TORICA_MoveAve<50> main_dps_altitude_platform_m(0);
TORICA_MoveAve<50> under_dps_altitude_platform_m(0);
TORICA_MoveAve<50> air_dps_altitude_platform_m(0);

// 気圧センサを用いた信頼できる対地高度
// 3つの気圧高度にそれぞれ移動平均をとってプラホを10mとし，中央値をとった値
#include "QuickStats.h"
float dps_altitude_lake_array_m[3];
QuickStats dps_altitude_lake_m;
// float dps_altitude_lake_m()
// {
//   if (dps_altitude_lake_array_m[0] > dps_altitude_lake_array_m[1])
//   {
//     if (dps_altitude_lake_array_m[1] > dps_altitude_lake_array_m[2])
//     {
//       return dps_altitude_lake_array_m[1];
//     }
//     else if (dps_altitude_lake_array_m[0] > dps_altitude_lake_array_m[2])
//     {
//       return dps_altitude_lake_array_m[2];
//     }
//     else
//     {
//       return dps_altitude_lake_array_m[0];
//     }
//   }
//   else
//   {
//     if (dps_altitude_lake_array_m[0] > dps_altitude_lake_array_m[2])
//     {
//       return dps_altitude_lake_array_m[0];
//     }
//     else if (dps_altitude_lake_array_m[1] > dps_altitude_lake_array_m[2])
//     {
//       return dps_altitude_lake_array_m[2];
//     }
//     else
//     {
//       return dps_altitude_lake_array_m[1];
//     }
//   }
// }

// 超音波高度(対地高度)
TORICA_MoveAve<3> filtered_under_urm_altitude_m(0.6);

#include "TORICA_MoveMedian.h"
// 気圧での対地高度と超音波での対地高度の差
// 100Hz(calculate)*5s = 500
TORICA_MoveMedian<500> altitude_dps_urm_offset_m(0);


// 気圧と超音波から推定した対地高度
float estimated_altitude_lake_m = const_platform_m;


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
volatile float data_air_sdp_airspeed_ms = 0;

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
  // LED初期化
  pinMode(LED_ICS, OUTPUT);
  pinMode(LED_Under, OUTPUT);
  pinMode(LED_Air, OUTPUT);

  // ネイティブポートからの書き込みでシリアルポートを使用するため，リセット直後はUARTを叩かない
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

  // UART初期化
  SerialGPS.begin(115200);  //SerialTWE
  SerialICS.begin(115200);  //SerialMainSD
  SerialAir.begin(460800);
  SerialUnder.begin(460800);
  SerialUSB.begin(115200);
  SerialTWE.print("loading...\n\n");

  // DPS310初期化
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

  // BNO055初期化
  if (!bno.begin()) {
    SerialUSB.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1) {
      digitalWrite(LED_ICS, HIGH);
      delay(100);
      digitalWrite(LED_ICS, LOW);
      delay(100);
    }
  }

  // 音声合成初期化
  Wire1.begin();
  Wire1.setClock(100000);

  // センサー・各基板の起動を待機
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
    func_100Hz();
  }

  // 合成音声読み上げ
  callout_status();

  // テレメトリダウンリンク(タイミングも関数内調整)
  TWE_downlink();
}


void func_100Hz() {
  //uint32_t time_us = micros();

  // GPS・操舵角・機体下電装部・エアデータ電装部読み取り
  polling_UART();

  // フライトフェーズ判断(メインの加速度も測定)
  determine_flight_phase();

  // 気圧センサ×3・超音波センサから高度推定
  calculate_altitude();

  // SDに記録(メインの気圧高度も測定)
  send_SD();

  // if (micros() - time_us > 9900) {  //MAX10000=100Hz
  //   SerialUSB.print("ISR100Hz_overrun!!!");
  // }
  // SerialUSB.print("ISR_us:");
  // SerialUSB.println(micros() - time_us);
}


void polling_UART() {
  //ICS
  data_ics_angle = ics.read_Angle();
  if (data_ics_angle > 0) {
    digitalWrite(LED_ICS, !digitalRead(LED_ICS));
  }

  //UnderSide
  static unsigned long int last_under_time_ms = 0;
  int readnum = Under_UART.readUART();
  int under_data_num = 4;
  if (readnum == under_data_num) {
    last_under_time_ms = millis();
    digitalWrite(LED_Under, !digitalRead(LED_Under));
    data_under_dps_pressure_hPa = Under_UART.UART_data[0];
    data_under_dps_temperature_deg = Under_UART.UART_data[1];
    data_under_dps_altitude_m = Under_UART.UART_data[2];
    data_under_urm_altitude_m = Under_UART.UART_data[3];
    filtered_under_dps_altitude_m.add(data_under_dps_altitude_m);
    if (flight_phase == PLATFORM) {
      under_dps_altitude_platform_m.add(data_under_dps_altitude_m);
    }
    filtered_under_urm_altitude_m.add(data_under_urm_altitude_m);
  }
  if (millis() - last_under_time_ms > 1000) {
    // 超音波高度のみ冗長系がないため，データが来なければ8mとして高度推定に渡す．
    // 測定範囲外のときは10mになり，9m以上でテイクオフ判断をするため故障時は8m
    // filtered_under_urm_altitude_m.add(8.0);
    // ToDo 明示的にis_aliveを作るべき．値の処理によって7変わる．
  }

  //AirData
  readnum = Air_UART.readUART();
  int air_data_num = 5;
  if (readnum == air_data_num) {
    digitalWrite(LED_Air, !digitalRead(LED_Air));
    data_air_dps_pressure_hPa = Air_UART.UART_data[0];
    data_air_dps_temperature_deg = Air_UART.UART_data[1];
    data_air_dps_altitude_m = Air_UART.UART_data[2];
    data_air_sdp_differentialPressure_Pa = Air_UART.UART_data[3];
    data_air_sdp_airspeed_ms = Air_UART.UART_data[4];
    filtered_airspeed_ms.add(data_air_sdp_airspeed_ms);
    filtered_air_dps_altitude_m.add(data_air_dps_altitude_m);
    if (flight_phase == PLATFORM) {
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
    }
  }
}


void determine_flight_phase() {
  //発進判定のため，IMU測定はここで行う
  read_main_bno();

  static unsigned long int takeoff_time_ms = 0;
  switch (flight_phase) {
    case PLATFORM:
      {
        static int over_urm_range_count = 0;
        if (filtered_under_urm_altitude_m.get() > 9.0) {
          over_urm_range_count++;
        } else {
          over_urm_range_count = 0;
        }
        bool over_urm_range = false;
        // 超音波が測定不能な状態が2秒以上続いたとき
        if (over_urm_range_count >= 200) {
          over_urm_range = true;
        }
        // 気圧センサにより下降したと判断したとき
        bool descending = estimated_altitude_lake_m < 10.0;
        if ((over_urm_range || descending) && millis() > 15000) {
          flight_phase = TAKEOFF;
          takeoff_time_ms = millis();
        }
        if (over_urm_range) {
          SerialTWE.print("\n\nover_urm_range\n\n");
        }
        if (descending) {
          SerialTWE.print("\n\ndescending\n\n");
        }
      }
      break;
    case TAKEOFF:
      // (ダイブするか知らんけど)ダイブ後に水平飛行に移ったとき(超音波の測定値が信頼できる状態のとき)
      // TAKEOFFで3秒待機することでHIGH_LEVELからMID_LEVELに瞬時に移行することを防ぐ
      if (millis() - takeoff_time_ms > 3000) {
        flight_phase = HIGH_LEVEL;
      }
      break;
    case HIGH_LEVEL:
      // 超音波が測定できるようになったとき
      if (filtered_under_urm_altitude_m.get() < 5.0 && estimated_altitude_lake_m < 5.0) {
        flight_phase = MID_LEVEL;
      }
      break;
    case MID_LEVEL:
      // 高度が1m以下になったとき(高度を高頻度で読み上げる)
      if (estimated_altitude_lake_m < 1.0) {
        flight_phase = LOW_LEVEL;
      }
      break;
    case LOW_LEVEL:
      break;
    default:
      break;
  }
}


void calculate_altitude() {
  // 100Hzで関数呼び出し
  dps_altitude_lake_array_m[0] = filtered_main_dps_altitude_m.get() - main_dps_altitude_platform_m.get() + const_platform_m;
  dps_altitude_lake_array_m[1] = filtered_under_dps_altitude_m.get() - under_dps_altitude_platform_m.get() + const_platform_m;
  dps_altitude_lake_array_m[2] = filtered_air_dps_altitude_m.get() - air_dps_altitude_platform_m.get() + const_platform_m;
  estimated_altitude_lake_m = dps_altitude_lake_m.median(dps_altitude_lake_array_m, 3);

  if (flight_phase == MID_LEVEL || flight_phase == LOW_LEVEL) {
    // 気圧センサが本来より低い値ならオフセットは正
    altitude_dps_urm_offset_m.add(filtered_under_urm_altitude_m.get() - estimated_altitude_lake_m);

    // 気圧センサが本来より低い値なら正のオフセットを足す
    estimated_altitude_lake_m += altitude_dps_urm_offset_m.get();
  }
}


void send_SD() {
  uint32_t time_ms = millis();
  static int loop_count = 0;
  if (loop_count == 0) {
    sprintf(UART_SD, "%d, %.2f,%.2f,%.2f,", time_ms,
            data_main_bno_accx_mss, data_main_bno_accy_mss, data_main_bno_accz_mss);
  } else if (loop_count == 1) {
    sprintf(UART_SD, "%.2f,%.2f,%.2f,%.2f, %.2f,%.2f,%.2f,",
            data_main_bno_qw, data_main_bno_qx, data_main_bno_qy, data_main_bno_qz,
            data_main_bno_roll, data_main_bno_pitch, data_main_bno_yaw);
  } else if (loop_count == 2) {

    // SDに書き込む直前で測定
    read_main_dps();

    sprintf(UART_SD, "%.2f,%d, %.2f,%.2f,%.2f, %.2f,%.2f,%.2f, %.2f,",
            estimated_altitude_lake_m, flight_phase,
            data_main_dps_pressure_hPa, data_main_dps_temperature_deg, data_main_dps_altitude_m,
            data_under_dps_pressure_hPa, data_under_dps_temperature_deg, data_under_dps_altitude_m, data_under_urm_altitude_m);

  } else if (loop_count == 3) {
    sprintf(UART_SD, "%.2f,%.2f,%.2f, %.2f,%.2f, %d,",
            data_air_dps_pressure_hPa, data_air_dps_temperature_deg, data_air_dps_altitude_m,
            data_air_sdp_differentialPressure_Pa, data_air_sdp_airspeed_ms,
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
}


void callout_status() {
  static uint32_t next_callout_time = 0;
  static bool force_call_alt = false;
  static int step_altitude_lake_m = 10;  //今これより下
  if (millis() >= next_callout_time) {
    if (flight_phase != PLATFORM) {
      bool call_speed = true;
      if (estimated_altitude_lake_m < step_altitude_lake_m - 1) {
        step_altitude_lake_m = (int)estimated_altitude_lake_m + 1;
        speaker.callout_altitude(step_altitude_lake_m);
        next_callout_time = millis() + 2400;
        call_speed = false;
      }
      if (force_call_alt) {
        speaker.callout_altitude(estimated_altitude_lake_m);
        next_callout_time = millis() + 2400;
        force_call_alt = false;
        call_speed = false;
      }
      if (call_speed) {
        speaker.callout_airspeed(filtered_airspeed_ms.get());
        next_callout_time = millis() + 1500;
        if (flight_phase == LOW_LEVEL) {
          force_call_alt = true;
        }
      }
    }
  }
}

void read_main_bno() {
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
  Quaternion qua(data_main_bno_qx, data_main_bno_qy, data_main_bno_qz, data_main_bno_qw);
  EulerAngles euler(qua.to_rotation_matrix());
  data_main_bno_roll = -(euler.first() * 180 / 3.1415);
  data_main_bno_pitch = euler.second() * 180 / 3.1415;
  data_main_bno_yaw = euler.third() * 180 / 3.1415;
}

void read_main_dps() {
  if (dps.temperatureAvailable() && dps.pressureAvailable()) {
    dps.getEvents(&temp_event, &pressure_event);
    data_main_dps_pressure_hPa = pressure_event.pressure;
    data_main_dps_temperature_deg = temp_event.temperature;
    data_main_dps_altitude_m = (powf(1013.25 / data_main_dps_pressure_hPa, 1 / 5.257) - 1) * (data_main_dps_temperature_deg + 273.15) / 0.0065;
    filtered_main_dps_altitude_m.add(data_main_dps_altitude_m);
    if (flight_phase == PLATFORM) {
      main_dps_altitude_platform_m.add(data_main_dps_altitude_m);
    }
  }
}

void TWE_downlink() {
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
    //sprintf(TWE_BUF, "%+06.2f\n", filtered_under_urm_altitude_m.get());
    //SerialTWE.print(TWE_BUF);
    SerialTWE.print("\n");
    TWE_downlink_type++;
    TWE_last_send_time = millis();
  } else if (TWE_downlink_type == 3 && millis() - TWE_last_send_time >= 400) {
    SerialTWE.print("AIR\n");
    sprintf(TWE_BUF, "pressure        temp    alt\n%+08.2f        %+06.2f  %+06.2f\n", data_air_dps_pressure_hPa, data_air_dps_temperature_deg, data_air_dps_altitude_m);
    SerialTWE.print(TWE_BUF);
    sprintf(TWE_BUF, "diffPressure    AirSpeed\n%+09.3f       %+06.2f\n", data_air_sdp_differentialPressure_Pa, data_air_sdp_airspeed_ms);
    SerialTWE.print(TWE_BUF);
    //sprintf(TWE_BUF, "                %+06.2f\n", filtered_airspeed_ms.get());
    //SerialTWE.print(TWE_BUF);
    SerialTWE.print("\n");
    TWE_downlink_type++;
    TWE_last_send_time = millis();
  } else if (TWE_downlink_type == 4 && millis() - TWE_last_send_time >= 400) {
    SerialTWE.print("ICS(joystick)\n");
    sprintf(TWE_BUF, "angle (center=7500)\n%d\n", data_ics_angle);
    SerialTWE.print(TWE_BUF);


    SerialTWE.print("estimated_altitude_lake_m\n");
    sprintf(TWE_BUF, "%+08.3f\n", estimated_altitude_lake_m);
    SerialTWE.print(TWE_BUF);


    //Reset downlink type
    TWE_downlink_type = 0;
    TWE_last_send_time = millis();
  }
}
