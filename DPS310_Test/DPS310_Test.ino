// This example shows how to read temperature/pressure

#include <Adafruit_DPS310.h>

Adafruit_DPS310 dps;

// Can also use SPI!
#define DPS310_CS 10

void setup() {
  SerialUSB.begin(115200);
  Wire.setClock(400000);
  while (!SerialUSB) delay(10);

  SerialUSB.println("DPS310");
  if (! dps.begin_I2C()) {             // Can pass in I2C address here
  //if (! dps.begin_SPI(DPS310_CS)) {  // If you want to use SPI
    SerialUSB.println("Failed to find DPS");
    while (1) yield();
  }
  SerialUSB.println("DPS OK!");

  dps.configurePressure(DPS310_64HZ, DPS310_64SAMPLES);
  dps.configureTemperature(DPS310_64HZ, DPS310_64SAMPLES);
}

void loop() {
  sensors_event_t temp_event, pressure_event;
  
  while (!dps.temperatureAvailable() || !dps.pressureAvailable()) {
    return; // wait until there's something to read
  }

  dps.getEvents(&temp_event, &pressure_event);
  SerialUSB.print(F("Temperature = "));
  SerialUSB.print(temp_event.temperature);
  SerialUSB.println(" *C");

  SerialUSB.print(F("Pressure = "));
  SerialUSB.print(pressure_event.pressure);
  SerialUSB.println(" hPa"); 

  SerialUSB.println();
}
