#include "OV7670.h"

class Adafruit_OV7670 {
 public:
  Adafruit_OV7670(uint8_t addr = OV7670_ADDR, int rst = -1, int xclk = -1, void *timer = NULL);
  ~Adafruit_OV7670();
  bool begin(void);
  uint8_t readRegister(uint8_t reg);
  void writeRegister(uint8_t reg, uint8_t value);

 private:
  const uint8_t i2c_address;
  const int     reset_pin;
  const int     xclk_pin;
  const void   *xclk_timer;
};
