#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define BNO055_SAMPLERATE_DELAY_MS (100)


Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);

void setup(void)
{
  SerialUSB.begin(115200);

  Wire.setClock(400000);
  while (!SerialUSB) delay(10);

  SerialUSB.println("Orientation Sensor Raw Data Test"); SerialUSB.println("");

  if(!bno.begin())
  {
    SerialUSB.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
}

void loop(void)
{
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  /* Display the floating point data */
  SerialUSB.print("X: ");
  SerialUSB.print(euler.x());
  SerialUSB.print(" Y: ");
  SerialUSB.print(euler.y());
  SerialUSB.print(" Z: ");
  SerialUSB.print(euler.z());
  SerialUSB.print("\t\t");

  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

  /* Display the floating point data */
  SerialUSB.print("X: ");
  SerialUSB.print(accel.x());
  SerialUSB.print(" Y: ");
  SerialUSB.print(accel.y());
  SerialUSB.print(" Z: ");
  SerialUSB.print(accel.z());
  SerialUSB.println("\t\t");

  /*
  // Quaternion data
  imu::Quaternion quat = bno.getQuat();
  SerialUSB.print("qW: ");
  SerialUSB.print(quat.w(), 4);
  SerialUSB.print(" qX: ");
  SerialUSB.print(quat.x(), 4);
  SerialUSB.print(" qY: ");
  SerialUSB.print(quat.y(), 4);
  SerialUSB.print(" qZ: ");
  SerialUSB.print(quat.z(), 4);
  SerialUSB.print("\t\t");
  */

  delay(BNO055_SAMPLERATE_DELAY_MS);
}
