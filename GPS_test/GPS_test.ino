void setup() {
 SerialUSB.begin(115200);
 Serial.begin(9600);
}

void loop() {
  if(SerialUSB.available()){
    char c = (char)SerialUSB.read();
    Serial.write(c);
  }
  if(Serial.available()){
    char c = (char)Serial.read();
    SerialUSB.write(c);
  }
}
