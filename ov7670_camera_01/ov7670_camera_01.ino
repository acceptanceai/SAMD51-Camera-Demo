// Cobbled OV7670 camera WIP for Grand Central.
// OV7670 MUST receive an 8-16 MHz clock signal on XCLK to run.
// This is NOT part of the SAMD PCC peripheral. Although there's
// a designated pin identified as PCC_XCLK on the Grand Central
// schematic (and maybe others), that signal must be provided by
// a timer peripheral. And if we want to stick by the schematic
// and not use a different pin for XCLK out, that REQUIRES using
// TCC1 for PCC_XCLK on Grand Central as it's the only timer
// output available on that pin (as TCC1/WO[1]). Other timers can
// only be used if physically wired to a different pin.
// ALTERNATELY: it MIGHT be possible to use the GCLK/IO[5] mux
// that the pin is capable of...looking at the other camera code,
// I think that might've been the plan...but reconfiguring GCLK5
// might be problematic if it's needed by other Arduino peripherals.
// SEE NOTES LATER. YES, GCLK5 IS PROBLEMATIC, MUST USE TCC1.

#include <Wire.h>
#include "wiring_private.h" // pinPeripheral() function

#define OV7670_ADDR 0x21
#define OV7670_REG_COM7        0x12    /* Control 7 */
#define OV7670_COM7_RESET      0x80    /* Register reset */
#define OV7670_REG_PID         0x0a    /* Product ID MSB */
#define OV7670_REG_VER         0x0b    /* Product ID LSB */

uint8_t _i2c_addr = OV7670_ADDR;

uint8_t readRegister(uint8_t reg) {
  Wire.beginTransmission(_i2c_addr);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(_i2c_addr, (uint8_t)1);
  return Wire.read();
}

void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(_i2c_addr);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// Need 8 MHz PWM out
// This goes to XCLK and runs continuously
// PCC XCLK *is* a designated peripheral pin
// If PCC peripheral is set up, this SHOULD do a thing
// No. PCC_XCLK, though declared as a pin on the schematic,
// is not really a peripheral function. We need to synthesize
// that signal ourselves.
// It's on PB19, which is also TCC1/WO[1] or GCLK/IO[5]
// SO, config up TCC1 for 8 MHz out on channel 1, set pinPeripheral to TCC

#define TIMER TCC1
#define GCLK_ID     TC4_GCLK_ID
#define GCM_ID      GCM_TC4_TC5

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // Configure GCLK channel for TCC1 to use GCLK1 (48 MHz) source
  GCLK->PCHCTRL[TCC1_GCLK_ID].bit.CHEN = 0; // Disable
  while (GCLK->PCHCTRL[TCC1_GCLK_ID].bit.CHEN) // Wait for disable
    ;
  // Select generator 1, enable channel, use .reg so it's an atomic op
  GCLK->PCHCTRL[TCC1_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK1 | GCLK_PCHCTRL_CHEN;
  while (!GCLK->PCHCTRL[TCC1_GCLK_ID].bit.CHEN) // Wait for enable
    ;

  TCC1->CTRLA.bit.ENABLE = 0; // Disable TCC before configuring
  while (TCC1->SYNCBUSY.bit.ENABLE)
    ;
  TCC1->CTRLA.bit.PRESCALER = TCC_CTRLA_PRESCALER_DIV1_Val; // 1:1 Prescale
  TCC1->WAVE.bit.WAVEGEN = TCC_WAVE_WAVEGEN_NPWM_Val; // Normal PWM mode
  while (TCC1->SYNCBUSY.bit.WAVE)
    ;
  // Datasheet claims 10-48 MHz clock input, with 24 MHz typical.
  // Reality though...24 MHz does NOT work, 8 MHz OK if that's what's available
  // These all seem to work:
  //  TCC1->PER.bit.PER = 5; // 8 MHz out (from 48 MHz source)
  //  TCC1->PER.bit.PER = 3; // 12 MHz out (from 48 MHz source)
  TCC1->PER.bit.PER = 2; // 16 MHz out (from 48 MHz source)
  while (TCC1->SYNCBUSY.bit.PER)
    ;
  //  TCC1->CC[1].bit.CC = 3; // 50% duty cycle @ 8 MHz
  //  TCC1->CC[1].bit.CC = 2; // 50% duty cycle @ 12 MHz
  TCC1->CC[1].bit.CC = 2; // 66% duty cycle @ 16 MHz
  while (TCC1->SYNCBUSY.bit.CC1)
    ;
  TCC1->CTRLA.bit.ENABLE = 1;
  while (TCC1->SYNCBUSY.bit.ENABLE)
    ;

  // Assign TCC peripheral to PCC_XCLK pin
  pinMode(PIN_PCC_XCLK, OUTPUT);
  pinPeripheral(PIN_PCC_XCLK, PIO_TIMER_ALT);

  // Careful about using analogWrite() after this

#if 0
  // Using GCLK5 for PCC_XCLK just Does Not Work. Things seem to lock up.
  // GCLK5 is configured for 1 MHz in Arduino core's startup.c
  // but I don't see where it's used after that.
  // Regardless, crashy if changed. Serial console doesn't work, etc.

  // GCLK5, feed off 48 MHz source, divide by 4 for 12 MHz, enable output
  GCLK->GENCTRL[5].reg = GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_DFLL_Val) | GCLK_GENCTRL_GENEN | GCLK_GENCTRL_DIV(4u) | GCLK_GENCTRL_OE;
  while ( GCLK->SYNCBUSY.bit.GENCTRL5 ); // Wait for sync

  pinPeripheral(PIN_PCC_XCLK, PIO_AC_CLK); // Use GCLK5
  // This DOES generate a 12 MHz wave on the XCLK pin, but everything else is then broken
#endif

  Serial.begin(9600);
  while (!Serial);
  delay(500);
  Serial.println("Hello");

  Wire.begin();
  Wire.setClock(100000); // Datasheet claims 400 KHz, but no, must be 100 KHz

  // Soft reset
  writeRegister(OV7670_REG_COM7, OV7670_COM7_RESET);
  delay(500);
  // SOFT RESET ALONE ISN'T ENOUGH. Camera's !reset pin should connect to
  // Arduino reset, or use a digital pin & toggle manually (low, high).

  uint8_t pid = readRegister(OV7670_REG_PID); // Should be 0x76
  uint8_t ver = readRegister(OV7670_REG_VER); // Should be 0x73

  Serial.println(pid, HEX);
  Serial.println(ver, HEX);

  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
}
