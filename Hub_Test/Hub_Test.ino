#define SerialGPS   Serial1
#define SerialTWE   Serial1

void setup() {
  // put your setup code here, to run once:

  SerialUSB.begin(115200);
  SerialGPS.begin(115200);
  SerialTWE.begin(115200);

}

void loop() {
  // put your main code here, to run repeatedly:
  SerialTWE.print("hello\r\n");
  while (SerialGPS.available()) {
    char c = (char)SerialGPS.read();
    SerialUSB.write(c);
  }
}
