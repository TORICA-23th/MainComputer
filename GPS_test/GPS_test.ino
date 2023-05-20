void setup() {
 SerialUSB.begin(115200);
 Serial1.begin(115200);
}

void loop() {
  if(SerialUSB.available()){
    char c = (char)SerialUSB.read();
    Serial1.write(c);
  }
  if(Serial1.available()){
    char c = (char)Serial1.read();
    SerialUSB.write(c);
  }
}
