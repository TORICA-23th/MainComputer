#include <Arduino.h>
#include <SPI.h>
#include <SD.h>

class TORICA_SD {
  public:
    TORICA_SD(int _cs_SD) {
      cs_SD = _cs_SD;
    }

    bool begin();
    void add_str(char str[64]);
    void flash();
    bool SDisActive = false;

  private:
    void new_file();
    void end();
    uint32_t file_time = 0;

    int cs_SD = LED_BUILTIN;
    char fileName[16];
    File dataFile;
    char SD_buf[2][32768];
    int SD_buf_index = 0;
    int SD_buf_count[2] = {0, 0};
};
