#include <Wire.h>
const int LED_ICS = 67;
void setup() {
  Wire1.begin();
  Wire1.setClock(100000);
}

void loop() {
  pinMode(LED_ICS, OUTPUT);
  Wire1.beginTransmission(0x2E); // スタートとスレーブアドレスを送る役割　（swの役割）

  char str[]= "aiueo";
  Wire1.write(str, strlen(str)*sizeof(char));
  Wire1.write('\r');

  Wire1.endTransmission();    // stop transmitting
  delay(1000);
    digitalWrite(LED_ICS, !digitalRead(LED_ICS));
}
