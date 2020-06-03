#include <Arduino.h>
#include <Wire.h>
#include "wiring_private.h" // pinPeripheral() function
#include "ov7670.h"

// Also provide: reset pin (or -1), timer periph.

OV7670::OV7670(uint8_t addr, int rst, int xclk_pin, void *timer) : i2c_address(addr & 0x7f), reset_pin(rst), xclk_pin(xclk_pin), xclk_timer(timer) {
}

OV7670::~OV7670() {
}

bool OV7670::begin() {

  static const struct {
    void *base;      // TC or TCC peripheral base address
    uint8_t GCLK_ID; // Timer ID for GCLK->PCHCTRL
  } timer[] = {
#if defined(TC0)
    {TC0, TC0_GCLK_ID},
#endif
#if defined(TC1)
    {TC1, TC1_GCLK_ID},
#endif
#if defined(TC2)
    {TC2, TC2_GCLK_ID},
#endif
#if defined(TC3)
    {TC3, TC3_GCLK_ID},
#endif
#if defined(TC4)
    {TC4, TC4_GCLK_ID},
#endif
#if defined(TC5)
    {TC5, TC5_GCLK_ID},
#endif
#if defined(TC6)
    {TC6, TC6_GCLK_ID},
#endif
#if defined(TC7)
    {TC7, TC7_GCLK_ID},
#endif
#if defined(TC8)
    {TC8, TC8_GCLK_ID},
#endif
#if defined(TC9)
    {TC9, TC9_GCLK_ID},
#endif
#if defined(TC10)
    {TC10, TC10_GCLK_ID},
#endif
#if defined(TC11)
    {TC11, TC11_GCLK_ID},
#endif
#if defined(TC12)
    {TC12, TC12_GCLK_ID},
#endif
    {NULL, 0}, // TC/TCC separator
#if defined(TCC0)
    { TCC0, TCC0_GCLK_ID },
#endif
#if defined(TCC1)
    { TCC1, TCC1_GCLK_ID },
#endif
#if defined(TCC2)
    { TCC2, TCC2_GCLK_ID },
#endif
#if defined(TCC3)
    { TCC3, TCC3_GCLK_ID },
#endif
#if defined(TCC4)
    { TCC4, TCC4_GCLK_ID },
#endif
  };

  uint8_t timer_list_index;
  bool is_tcc = false;
  for(timer_list_index=0; (timer_list_index < sizeof timer / sizeof timer[0]) && (timer[timer_list_index].base != xclk_timer); timer_list_index++) {
    if(!timer[timer_list_index].base) {
      is_tcc = true;
      continue;
    }
  }
  if(timer_list_index >= sizeof timer / sizeof timer[0]) {
    return false;
  }

  uint8_t id = timer[timer_list_index].GCLK_ID;

  // Configure GCLK channel for timer to use GCLK1 (48 MHz) source
  GCLK->PCHCTRL[id].bit.CHEN = 0; // Disable
  while (GCLK->PCHCTRL[id].bit.CHEN) // Wait for disable
    ;
  // Select generator 1, enable channel, use .reg so it's an atomic op
  GCLK->PCHCTRL[id].reg = GCLK_PCHCTRL_GEN_GCLK1 | GCLK_PCHCTRL_CHEN;
  while (!GCLK->PCHCTRL[id].bit.CHEN) // Wait for enable
    ;

  pinMode(xclk_pin, OUTPUT);
  if(is_tcc) {
    Tcc *tcc = (Tcc *)timer[timer_list_index].base;

    tcc->CTRLA.bit.ENABLE = 0; // Disable TCC before configuring
    while (tcc->SYNCBUSY.bit.ENABLE)
      ;
    tcc->CTRLA.bit.PRESCALER = TCC_CTRLA_PRESCALER_DIV1_Val; // 1:1 Prescale
    tcc->WAVE.bit.WAVEGEN = TCC_WAVE_WAVEGEN_NPWM_Val; // Normal PWM mode
    while (tcc->SYNCBUSY.bit.WAVE)
      ;
    // Datasheet claims 10-48 MHz clock input, with 24 MHz typical.
    // Reality though...24 MHz does NOT work, 8 MHz OK if that's what's available
    // These all seem to work:
    //  tcc->PER.bit.PER = 5; // 8 MHz out (from 48 MHz source)
    //  tcc->PER.bit.PER = 3; // 12 MHz out (from 48 MHz source)
    tcc->PER.bit.PER = 2; // 16 MHz out (from 48 MHz source)
    while (tcc->SYNCBUSY.bit.PER)
      ;
    //  tcc->CC[1].bit.CC = 3; // 50% duty cycle @ 8 MHz
    //  tcc->CC[1].bit.CC = 2; // 50% duty cycle @ 12 MHz
    tcc->CC[1].bit.CC = 2; // 66% duty cycle @ 16 MHz
    while (tcc->SYNCBUSY.bit.CC1)
      ;
    tcc->CTRLA.bit.ENABLE = 1;
    while (tcc->SYNCBUSY.bit.ENABLE)
      ;
    pinPeripheral(xclk_pin, PIO_TIMER_ALT);
    // Note: sometimes might want PIO_TCC_PDEC instead, how to establish that?
  } else {
    Tc *tc = (Tc *)timer[timer_list_index].base;
    pinPeripheral(xclk_pin, PIO_TIMER);
  }


  if(reset_pin >= 0) {
    pinMode(reset_pin, OUTPUT);
    digitalWrite(reset_pin, LOW);
    delay(100);
    digitalWrite(reset_pin, HIGH);
  }

  Wire.begin();
  Wire.setClock(100000); // Datasheet claims 400 KHz, but no, must be 100 KHz

  return true;
}

uint8_t OV7670::readRegister(uint8_t reg) {
  Wire.beginTransmission(i2c_address);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(i2c_address, (uint8_t)1);
  return Wire.read();
}

void OV7670::writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(i2c_address);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}
