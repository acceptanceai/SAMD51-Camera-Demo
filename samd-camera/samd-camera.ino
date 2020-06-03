#include <Arduino.h>

#include <Wire.h>
#include <SPI.h>
#include "wiring_private.h"

#include "ov7670.h"
#include "ili9341.h"

#include <Adafruit_ZeroDMA.h>
#include "utility/dma.h"

#define TFT_DC 9
#define TFT_CS 10
#define TFT_RST 6

ov7670 cam;
ili9341 tft(TFT_CS, TFT_DC, TFT_RST, &SPI);

Adafruit_ZeroDMA pccDMA;
DmacDescriptor *pccDesc1;
ZeroDMAstatus    stat; // DMA status codes returned by some functions

#define DATA_LENGTH (320*240/sizeof(uint16_t))
uint32_t datamem[DATA_LENGTH];

void empty(Adafruit_ZeroDMA *dma) {}


void startPCC() {
  stat = pccDMA.startJob();
  //detachInterrupt(PIN_PCC_DEN1);
}

void setup() {

  Serial.begin(115200);
  while (!Serial);
  delay(500);
Serial.println("A");
  // Currently only works with cache disabled
  CMCC->CTRL.bit.CEN = 0; // disable
  while (CMCC->SR.bit.CSTS);
  CMCC->MAINT0.bit.INVALL = 1; // invalidate all

Serial.println("B");
  SPI.begin();
Serial.println("C");

  tft.begin();
  tft.setRotation(1);


Serial.println("D");
  //begin PCC
  MCLK->APBDMASK.reg |= MCLK_APBDMASK_PCC;

  pinPeripheral(PIN_PCC_CLK, PIO_PCC);
  pinPeripheral(PIN_PCC_DEN1, PIO_PCC);
  pinPeripheral(PIN_PCC_DEN2, PIO_PCC);
  pinPeripheral(PIN_PCC_D0, PIO_PCC);
  pinPeripheral(PIN_PCC_D1, PIO_PCC);
  pinPeripheral(PIN_PCC_D2, PIO_PCC);
  pinPeripheral(PIN_PCC_D3, PIO_PCC);
  pinPeripheral(PIN_PCC_D4, PIO_PCC);
  pinPeripheral(PIN_PCC_D5, PIO_PCC);
  pinPeripheral(PIN_PCC_D6, PIO_PCC);
  pinPeripheral(PIN_PCC_D7, PIO_PCC);

  PCC->MR.reg = PCC_MR_ISIZE(0x00)       //8 bit data
                | PCC_MR_DSIZE(0x02)      //32 bits per transfer
                | PCC_MR_CID(0x01);          //reset on falling edge of DEN1 (vsync)
  PCC->MR.bit.PCEN = 1;                    //enable
Serial.println("E");

  //PCC DMA channel
  pccDMA.setTrigger(PCC_DMAC_ID_RX);
  pccDMA.setAction(DMA_TRIGGER_ACTON_BEAT);
  stat = pccDMA.allocate();
Serial.println("F");

  pccDesc1 = pccDMA.addDescriptor(
               (void *)(&PCC->RHR.reg),          // move data from here
               datamem,                             // to here
               DATA_LENGTH,                      // this many...
               DMA_BEAT_SIZE_WORD,               // bytes/hword/words
               false,                            // increment source addr?
               true);                            // increment dest addr?

  pccDMA.setCallback(empty);
  //pccDesc1->BTCTRL.bit.BLOCKACT = DMA_BLOCK_ACTION_INT;
Serial.println("G"); Serial.flush();

// Appears to crash here...
// Reboots 1-2 times before just stopping
// Wait - GCLK5 is *already* in use by SAMD51 core (as 1M clock)
  GCLK->GENCTRL[5].reg = GCLK_GENCTRL_SRC(GCLK_GENCTRL_SRC_DPLL0_Val) | // DPLL0 clock source = F_CPU (120 MHz)
                         GCLK_GENCTRL_IDC |    // Improve duty cycle
                         GCLK_GENCTRL_DIVSEL | // Divide by 2^(DIV+1) = 8
                         GCLK_GENCTRL_OE |     // Output enable
                         GCLK_GENCTRL_DIV(2) | // Division bits
                         GCLK_GENCTRL_GENEN;   // Enable
Serial.println("G1"); Serial.flush();

  while ( GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_GENCTRL5);
Serial.println("G2"); Serial.flush();
  pinPeripheral(PIN_PCC_XCLK, PIO_AC_CLK);
Serial.println("H"); Serial.flush();

  cam.begin();

Serial.println("I");
  attachInterrupt(PIN_PCC_DEN1, startPCC, FALLING);
Serial.println("J");
}

uint32_t foo = 0;
void loop() {
  Serial.println(foo++);
  tft.window (0, 0, 320, 240);
  tft.writeCommand(0x2C);

  uint8_t *ptr = (uint8_t *)datamem;
  for (uint32_t i = 0; i < DATA_LENGTH * 4; i++) {
    SERCOM7->SPI.DATA.bit.DATA = *ptr++; // Writing data into Data register

    while ( SERCOM7->SPI.INTFLAG.bit.DRE == 0 );

  }
  tft.endWrite();

}
