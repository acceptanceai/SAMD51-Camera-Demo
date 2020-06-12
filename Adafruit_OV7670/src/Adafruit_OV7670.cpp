#include "Adafruit_OV7670.h"
#include "wiring_private.h" // pinPeripheral() function
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ZeroDMA.h>

Adafruit_ZeroDMA pccDMA;
DmacDescriptor  *pccDesc1;
ZeroDMAstatus    stat;
uint16_t         datamem[320 * 240]; // ~150KB

Adafruit_OV7670::Adafruit_OV7670(uint8_t addr, int rst, int xclk_pin,
                                 void *timer, TwoWire *twi)
    : wire(twi), i2c_address(addr & 0x7f), reset_pin(rst), xclk_pin(xclk_pin),
      xclk_timer(timer) {}

Adafruit_OV7670::~Adafruit_OV7670() {}

static const struct {
  uint8_t reg;
  uint8_t value;
} ov7670_init[] = {
    {OV7670_REG_TSLB, OV7670_TSLB_YLAST},
    {OV7670_REG_COM15, OV7670_COM15_RGB565},
    {OV7670_REG_COM7, OV7670_COM7_SIZE_QVGA | OV7670_COM7_RGB},
    {OV7670_REG_HREF, 0x80},
    {OV7670_REG_HSTART, 0x16},
    {OV7670_REG_COM10, OV7670_COM10_VS_NEG}, // Neg VSYNC (req by SAMD51 PCC)
    {OV7670_REG_HSTOP, 0x04},
    {OV7670_REG_VSTART, 0x03},
    {OV7670_REG_VSTOP, 0x7B},
    {OV7670_REG_VREF, 0x06},
    {OV7670_REG_COM3, OV7670_COM3_SCALEEN | OV7670_COM3_DCWEN},
    {OV7670_REG_COM14, 0x00},
    {OV7670_REG_SCALING_XSC, 0x00},
    {OV7670_REG_SCALING_YSC, 0x01},
    {OV7670_REG_SCALINGDCW, 0x11},
    {OV7670_REG_SCALINGPCLK, 0x09},
    {OV7670_REG_SPD, 0x02},
    {OV7670_REG_CLKRC, 0x01},
    {OV7670_REG_SLOP, 0x20},
    {OV7670_REG_GAM_BASE, 0x1C},
    {OV7670_REG_GAM_BASE + 1, 0x28},
    {OV7670_REG_GAM_BASE + 2, 0x3C},
    {OV7670_REG_GAM_BASE + 3, 0x55},
    {OV7670_REG_GAM_BASE + 4, 0x68},
    {OV7670_REG_GAM_BASE + 5, 0x76},
    {OV7670_REG_GAM_BASE + 6, 0x80},
    {OV7670_REG_GAM_BASE + 7, 0x88},
    {OV7670_REG_GAM_BASE + 8, 0x8F},
    {OV7670_REG_GAM_BASE + 9, 0x96},
    {OV7670_REG_GAM_BASE + 10, 0xA3},
    {OV7670_REG_GAM_BASE + 11, 0xAF},
    {OV7670_REG_GAM_BASE + 12, 0xC4},
    {OV7670_REG_GAM_BASE + 13, 0xD7},
    {OV7670_REG_GAM_BASE + 14, 0xE8},
    {OV7670_REG_COM8,
     OV7670_COM8_FASTAEC | OV7670_COM8_AECSTEP | OV7670_COM8_BANDING},
    {OV7670_REG_GAIN, 0x00},
    {OV7670_COM2_SSLEEP, 0x00},
    {OV7670_REG_COM4, 0x00},
    {OV7670_REG_COM9, 0x20}, // Max AGC value
    {OV7670_REG_BD50MAX, 0x05},
    {OV7670_REG_BD60MAX, 0x07},
    {OV7670_REG_AEW, 0x75},
    {OV7670_REG_AEB, 0x63},
    {OV7670_REG_VPT, 0xA5},
    {OV7670_REG_HAECC1, 0x78},
    {OV7670_REG_HAECC2, 0x68},
    {0xA1, 0x03},              // Reserved register?
    {OV7670_REG_HAECC3, 0xDF}, // Histogram-based AEC/AGC setup
    {OV7670_REG_HAECC4, 0xDF},
    {OV7670_REG_HAECC5, 0xF0},
    {OV7670_REG_HAECC6, 0x90},
    {OV7670_REG_HAECC7, 0x94},
    {OV7670_REG_COM8, OV7670_COM8_FASTAEC | OV7670_COM8_AECSTEP |
                          OV7670_COM8_BANDING | OV7670_COM8_AGC |
                          OV7670_COM8_AEC},
    {OV7670_REG_COM5, 0x61},
    {OV7670_REG_COM6, 0x4B},
    {0x16, 0x02},            // Reserved register?
    {OV7670_REG_MVFP, 0x07}, // 0x07,
    {OV7670_REG_ADCCTR0, 0x02},
    {OV7670_REG_ADCCTR2, 0x91},
    {0x29, 0x07}, // Reserved register?
    {OV7670_REG_CHLF, 0x0B},
    {0x35, 0x0B}, // Reserved register?
    {OV7670_REG_ADC, 0x1D},
    {OV7670_REG_ACOM, 0x71},
    {OV7670_REG_OFON, 0x2A},
    {OV7670_REG_COM12, 0x78},
    {0x4D, 0x40}, // Reserved register?
    {0x4E, 0x20}, // Reserved register?
    {OV7670_REG_GFIX, 0x5D},
    {OV7670_REG_DBLV, 0x40},
    {OV7670_REG_REG74, 0x19},
    {0x8D, 0x4F}, // Reserved register?
    {0x8E, 0x00}, // Reserved register?
    {0x8F, 0x00}, // Reserved register?
    {0x90, 0x00}, // Reserved register?
    {0x91, 0x00}, // Reserved register?
    {OV7670_REG_DM_LNL, 0x00},
    {0x96, 0x00}, // Reserved register?
    {0x9A, 0x80}, // Reserved register?
    {0xB0, 0x84}, // Reserved register?
    {OV7670_REG_ABLC1, 0x0C},
    {0xB2, 0x0E}, // Reserved register?
    {OV7670_REG_THL_ST, 0x82},
    {0xB8, 0x0A}, // Reserved register?
    {OV7670_REG_AWBC1, 0x14},
    {OV7670_REG_AWBC2, 0xF0},
    {OV7670_REG_AWBC3, 0x34},
    {OV7670_REG_AWBC4, 0x58},
    {OV7670_REG_AWBC5, 0x28},
    {OV7670_REG_AWBC6, 0x3A},
    {0x59, 0x88}, // Reserved register?
    {0x5A, 0x88}, // Reserved register?
    {0x5B, 0x44}, // Reserved register?
    {0x5C, 0x67}, // Reserved register?
    {0x5D, 0x49}, // Reserved register?
    {0x5E, 0x0E}, // Reserved register?
    {OV7670_REG_LCC3, 0x04},
    {OV7670_REG_LCC4, 0x20},
    {OV7670_REG_LCC5, 0x05},
    {OV7670_REG_LCC6, 0x04},
    {OV7670_REG_LCC7, 0x08},
    {OV7670_REG_AWBCTR3, 0x0A},
    {OV7670_REG_AWBCTR2, 0x55},
    {OV7670_REG_MTX1, 0x80},
    {OV7670_REG_MTX2, 0x80},
    {OV7670_REG_MTX3, 0x00},
    {OV7670_REG_MTX4, 0x22},
    {OV7670_REG_MTX5, 0x5E},
    {OV7670_REG_MTX6, 0x80}, // 0x40?
    {OV7670_REG_AWBCTR1, 0x11},
    {OV7670_REG_AWBCTR0, 0x9F}, // Or use 0x9E for advance AWB
    {OV7670_REG_BRIGHT, 0x00},
    {OV7670_REG_CONTRAS, 0x40},
    {OV7670_REG_CONTRAS_CTR, 0x80}, // 0x40?
};

void startPCC() {
  stat = pccDMA.startJob();
  //detachInterrupt(PIN_PCC_DEN1);
}

bool Adafruit_OV7670::begin() {

  static const struct {
    void *base;      ///< TC or TCC peripheral base address
    uint8_t GCLK_ID; ///< Timer ID for GCLK->PCHCTRL
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
    {TCC0, TCC0_GCLK_ID},
#endif
#if defined(TCC1)
    {TCC1, TCC1_GCLK_ID},
#endif
#if defined(TCC2)
    {TCC2, TCC2_GCLK_ID},
#endif
#if defined(TCC3)
    {TCC3, TCC3_GCLK_ID},
#endif
#if defined(TCC4)
    {TCC4, TCC4_GCLK_ID},
#endif
  };

  uint8_t timer_list_index;
  bool is_tcc = false;
  for (timer_list_index = 0;
       (timer_list_index < sizeof timer / sizeof timer[0]) &&
       (timer[timer_list_index].base != xclk_timer);
       timer_list_index++) {
    if (!timer[timer_list_index].base) {
      is_tcc = true;
      continue;
    }
  }
  if (timer_list_index >= sizeof timer / sizeof timer[0]) {
    return false;
  }

  uint8_t id = timer[timer_list_index].GCLK_ID;

  // Configure GCLK channel for timer to use GCLK1 (48 MHz) source
  GCLK->PCHCTRL[id].bit.CHEN = 0;    // Disable
  while (GCLK->PCHCTRL[id].bit.CHEN) // Wait for disable
    ;
  // Select generator 1, enable channel, use .reg so it's an atomic op
  GCLK->PCHCTRL[id].reg = GCLK_PCHCTRL_GEN_GCLK1 | GCLK_PCHCTRL_CHEN;
  while (!GCLK->PCHCTRL[id].bit.CHEN) // Wait for enable
    ;

  pinMode(xclk_pin, OUTPUT);
  if (is_tcc) {
    Tcc *tcc = (Tcc *)timer[timer_list_index].base;

    tcc->CTRLA.bit.ENABLE = 0; // Disable TCC before configuring
    while (tcc->SYNCBUSY.bit.ENABLE)
      ;
    tcc->CTRLA.bit.PRESCALER = TCC_CTRLA_PRESCALER_DIV1_Val; // 1:1 Prescale
    tcc->WAVE.bit.WAVEGEN = TCC_WAVE_WAVEGEN_NPWM_Val;       // Normal PWM mode
    while (tcc->SYNCBUSY.bit.WAVE)
      ;
    // Datasheet claims 10-48 MHz clock input, with 24 MHz typical.
    // Reality though...24 MHz does NOT work, 8 MHz OK if that's what's
    // available These all seem to work:
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
    // Will this need to be an obscure arg to the constructor? That's super
    // specific to SAMD. So maybe the timer-related arguments to constuctor
    // go in a platform-specific struct or something.
  } else {
    Tc *tc = (Tc *)timer[timer_list_index].base;

    // Do regular Timer/Counter (vs TCC) setup here

    pinPeripheral(xclk_pin, PIO_TIMER);
  }

  // Unsure of camera startup time from beginning of input clock.
  // Let's guess it's similar to tS:REG (300 ms) from datasheet.
  delay(300);

  wire->begin();
  wire->setClock(100000); // Datasheet claims 400 KHz, but no, must be 100 KHz

  if (reset_pin >= 0) {
    // Hard reset
    pinMode(reset_pin, OUTPUT);
    digitalWrite(reset_pin, LOW);
    delay(1);
    digitalWrite(reset_pin, HIGH);
  } else {
    // Soft reset, doesn't seem reliable, might just need more delay?
    writeRegister(OV7670_REG_COM7, OV7670_COM7_RESET);
  }
  delay(1); // Datasheet: tS:RESET = 1 ms

  // Issue init sequence to camera
  for (int i = 0; i < (sizeof ov7670_init / sizeof ov7670_init[0]); i++) {
    writeRegister(ov7670_init[i].reg, ov7670_init[i].value);
  }
  delay(300); // Datasheet: tS:REG = 300 ms (settling time = 10 frames)

  // Set up PCC peripheral --------

  MCLK->APBDMASK.reg |= MCLK_APBDMASK_PCC;

  pinPeripheral(PIN_PCC_CLK, PIO_PCC);  // Camera PCLK
  pinPeripheral(PIN_PCC_DEN1, PIO_PCC); // Camera VSYNC
  pinPeripheral(PIN_PCC_DEN2, PIO_PCC); // Camera HSYNC
  pinPeripheral(PIN_PCC_D0, PIO_PCC);
  pinPeripheral(PIN_PCC_D1, PIO_PCC);
  pinPeripheral(PIN_PCC_D2, PIO_PCC);
  pinPeripheral(PIN_PCC_D3, PIO_PCC);
  pinPeripheral(PIN_PCC_D4, PIO_PCC);
  pinPeripheral(PIN_PCC_D5, PIO_PCC);
  pinPeripheral(PIN_PCC_D6, PIO_PCC);
  pinPeripheral(PIN_PCC_D7, PIO_PCC);

  PCC->MR.reg = PCC_MR_ISIZE(0x00) // 8 bit data
              | PCC_MR_DSIZE(0x02) // 32 bits per transfer
              | PCC_MR_CID(0x01);  // reset on falling edge of DEN1 (vsync)
  PCC->MR.bit.PCEN = 1;            // enable

  // Set up PCC DMA ---------------

  memset(datamem, 0, sizeof datamem);

  pccDMA.setTrigger(PCC_DMAC_ID_RX);
  pccDMA.setAction(DMA_TRIGGER_ACTON_BEAT);
  stat = pccDMA.allocate();

  pccDesc1 = pccDMA.addDescriptor(
               (void *)(&PCC->RHR.reg), // move data from here
               datamem,                 // to here
               320 * 240,               // this many...
               DMA_BEAT_SIZE_WORD,      // bytes/hword/words
               false,                   // increment source addr?
               true);                   // increment dest addr?

  attachInterrupt(PIN_PCC_DEN1, startPCC, FALLING);

  return true;
}

uint8_t Adafruit_OV7670::readRegister(uint8_t reg) {
  wire->beginTransmission(i2c_address);
  wire->write(reg);
  wire->endTransmission();
  wire->requestFrom(i2c_address, (uint8_t)1);
  return wire->read();
}

void Adafruit_OV7670::writeRegister(uint8_t reg, uint8_t value) {
  wire->beginTransmission(i2c_address);
  wire->write(reg);
  wire->write(value);
  wire->endTransmission();
}
