#define OV7670_ADDR 0x21 // Default I2C address if unspecified

#define OV7670_REG_COM7        0x12    /* Control 7 */
#define OV7670_COM7_RESET      0x80    /* Register reset */
#define OV7670_REG_PID         0x0a    /* Product ID MSB */
#define OV7670_REG_VER         0x0b    /* Product ID LSB */

class OV7670 {
 public:
  OV7670(uint8_t addr = OV7670_ADDR, int rst = -1, int xclk = -1, void *timer = NULL);
  ~OV7670();
  bool begin(void);
  uint8_t readRegister(uint8_t reg);
  void writeRegister(uint8_t reg, uint8_t value);

 private:
  const uint8_t i2c_address;
  const int     reset_pin;
  const int     xclk_pin;
  const void   *xclk_timer;
};
