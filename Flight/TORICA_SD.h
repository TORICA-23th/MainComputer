#include <Arduino.h>
#include <SPI.h>
#include <SD.h>

class TORICA_SD {
  public:
    TORICA_SD(int _cs_SD) {
      cs_SD = _cs_SD;
    }

    bool begin();
    void add_str(String str);
    void flash();
    bool SDisActive = false;

  private:

    int cs_SD = LED_BUILTIN;
    char fileName[16];
    File dataFile;
    String SD_buf[2];
    int SD_buf_index = 0;
};
