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

// Oh! Looks like GCLK5 is used to drive the PLLs which in turn
// RUN THE CORE CLOCK so yeah that's kind of a bad thing to change.


#include <Wire.h>
#include "Adafruit_OV7670.h"


// Need 8 MHz PWM out
// This goes to XCLK and runs continuously
// PCC XCLK *is* a designated peripheral pin
// If PCC peripheral is set up, this SHOULD do a thing
// No. PCC_XCLK, though declared as a pin on the schematic,
// is not really a peripheral function. We need to synthesize
// that signal ourselves.
// It's on PB19, which is also TCC1/WO[1] or GCLK/IO[5]
// SO, config up TCC1 for 8 MHz out on channel 1, set pinPeripheral to TCC

Adafruit_OV7670 cam(OV7670_ADDR, -1, PIN_PCC_XCLK, TCC1);

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);


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

  cam.begin();

  // Soft reset
  cam.writeRegister(OV7670_REG_COM7, OV7670_COM7_RESET);
  delay(500);
  // SOFT RESET ALONE ISN'T ENOUGH. Camera's !reset pin should connect to
  // Arduino reset, or use a digital pin & toggle manually (low, high).

  uint8_t pid = cam.readRegister(OV7670_REG_PID); // Should be 0x76
  uint8_t ver = cam.readRegister(OV7670_REG_VER); // Should be 0x73

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
