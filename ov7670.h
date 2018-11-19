/*
 * ov7670.h
 *
 *  Created on: Apr 4, 2018
 *      Author: deanm
 */

#ifndef OV7670_H_
#define OV7670_H_

#define Wire Wire1

#define CHANGE_REG_NUM 115
extern const uint8_t change_reg[CHANGE_REG_NUM][2];

#define OV7670_ADDR 0x21

#define OV7670_REG_GAIN        0x00    /* Gain lower 8 bits (rest in vref) */
#define OV7670_REG_BLUE        0x01    /* blue gain */
#define OV7670_REG_RED         0x02    /* red gain */
#define OV7670_REG_VREF        0x03    /* Pieces of GAIN, VSTART, VSTOP */
#define OV7670_REG_COM1        0x04    /* Control 1 */
#define OV7670_COM1_CCIR656    0x40    /* CCIR656 enable */
#define OV7670_REG_BAVE        0x05    /* U/B Average level */
#define OV7670_REG_GbAVE       0x06    /* Y/Gb Average level */
#define OV7670_REG_AECHH       0x07    /* AEC MS 5 bits */
#define OV7670_REG_RAVE        0x08    /* V/R Average level */
#define OV7670_REG_COM2        0x09    /* Control 2 */
#define OV7670_COM2_SSLEEP     0x10    /* Soft sleep mode */
#define OV7670_REG_PID         0x0a    /* Product ID MSB */
#define OV7670_REG_VER         0x0b    /* Product ID LSB */
#define OV7670_REG_COM3        0x0c    /* Control 3 */
#define OV7670_COM3_SWAP       0x40    /* Byte swap */
#define OV7670_COM3_SCALEEN    0x08    /* Enable scaling */
#define OV7670_COM3_DCWEN      0x04    /* Enable downsamp/crop/window */
#define OV7670_REG_COM4        0x0d    /* Control 4 */
#define OV7670_REG_COM5        0x0e    /* All "reserved" */
#define OV7670_REG_COM6        0x0f    /* Control 6 */
#define OV7670_REG_AECH        0x10    /* More bits of AEC value */
#define OV7670_REG_CLKRC       0x11    /* Clocl control */
#define OV7670_CLK_EXT         0x40    /* Use external clock directly */
#define OV7670_CLK_SCALE       0x3f    /* Mask for internal clock scale */
#define OV7670_REG_COM7        0x12    /* Control 7 */
#define OV7670_COM7_RESET      0x80    /* Register reset */
#define OV7670_COM7_FMT_MASK   0x38
#define OV7670_COM7_FMT_VGA    0x00
#define OV7670_COM7_FMT_CIF    0x20    /* CIF format */
#define OV7670_COM7_FMT_QVGA   0x10    /* QVGA format */
#define OV7670_COM7_FMT_QCIF   0x08    /* QCIF format */
#define OV7670_COM7_RGB        0x04    /* bits 0 and 2 - RGB format */
#define OV7670_COM7_YUV        0x00    /* YUV */
#define OV7670_COM7_BAYER      0x01    /* Bayer format */
#define OV7670_COM7_PBAYER     0x05    /* "Processed bayer" */
#define OV7670_REG_COM8        0x13    /* Control 8 */
#define OV7670_COM8_FASTAEC    0x80    /* Enable fast AGC/AEC */
#define OV7670_COM8_AECSTEP    0x40    /* Unlimited AEC step size */
#define OV7670_COM8_BFILT      0x20    /* Band filter enable */
#define OV7670_COM8_AGC        0x04    /* Auto gain enable */
#define OV7670_COM8_AWB        0x02    /* White balance enable */
#define OV7670_COM8_AEC        0x01    /* Auto exposure enable */
#define OV7670_REG_COM9        0x14    /* Control 9  - gain ceiling */
#define OV7670_REG_COM10       0x15    /* Control 10 */
#define OV7670_COM10_HSYNC     0x40    /* HSYNC instead of HREF */
#define OV7670_COM10_PCLK_HB   0x20    /* Suppress PCLK on horiz blank */
#define OV7670_COM10_HREF_REV  0x08    /* Reverse HREF */
#define OV7670_COM10_VS_LEAD   0x04    /* VSYNC on clock leading edge */
#define OV7670_COM10_VS_NEG    0x02    /* VSYNC negative */
#define OV7670_COM10_HS_NEG    0x01    /* HSYNC negative */
#define OV7670_REG_HSTART      0x17    /* Horiz start high bits */
#define OV7670_REG_HSTOP       0x18    /* Horiz stop high bits */
#define OV7670_REG_VSTART      0x19    /* Vert start high bits */
#define OV7670_REG_VSTOP       0x1a    /* Vert stop high bits */
#define OV7670_REG_PSHFT       0x1b    /* Pixel delay after HREF */
#define OV7670_REG_MIDH        0x1c    /* Manuf. ID high */
#define OV7670_REG_MIDL        0x1d    /* Manuf. ID low */
#define OV7670_REG_MVFP        0x1e    /* Mirror / vflip */
#define OV7670_MVFP_MIRROR     0x20    /* Mirror image */
#define OV7670_MVFP_FLIP       0x10    /* Vertical flip */
#define OV7670_REG_AEW         0x24    /* AGC upper limit */
#define OV7670_REG_AEB         0x25    /* AGC lower limit */
#define OV7670_REG_VPT         0x26    /* AGC/AEC fast mode op region */
#define OV7670_REG_HSYST       0x30    /* HSYNC rising edge delay */
#define OV7670_REG_HSYEN       0x31    /* HSYNC falling edge delay */
#define OV7670_REG_HREF        0x32    /* HREF pieces */
#define OV7670_REG_TSLB        0x3a    /* lots of stuff */
#define OV7670_TSLB_YLAST      0x04    /* UYVY or VYUY - see com13 */
#define OV7670_REG_COM11       0x3b    /* Control 11 */
#define OV7670_COM11_NIGHT     0x80    /* NIght mode enable */
#define OV7670_COM11_NMFR      0x60    /* Two bit NM frame rate */
#define OV7670_COM11_HZAUTO    0x10    /* Auto detect 50/60 Hz */
#define OV7670_COM11_50HZ      0x08    /* Manual 50Hz select */
#define OV7670_COM11_EXP       0x02
#define OV7670_REG_COM12       0x3c    /* Control 12 */
#define OV7670_COM12_HREF      0x80    /* HREF always */
#define OV7670_REG_COM13       0x3d    /* Control 13 */
#define OV7670_COM13_GAMMA     0x80    /* Gamma enable */
#define OV7670_COM13_UVSAT     0x40    /* UV saturation auto adjustment */
#define OV7670_COM13_UVSWAP    0x01    /* V before U - w/TSLB */
#define OV7670_REG_COM14       0x3e    /* Control 14 */
#define OV7670_COM14_DCWEN     0x10    /* DCW/PCLK-scale enable */
#define OV7670_REG_EDGE        0x3f    /* Edge enhancement factor */
#define OV7670_REG_COM15       0x40    /* Control 15 */
#define OV7670_COM15_R10F0     0x00    /* Data range 10 to F0 */
#define OV7670_COM15_R01FE     0x80    /*            01 to FE */
#define OV7670_COM15_R00FF     0xc0    /*            00 to FF */
#define OV7670_COM15_RGB565    0x10    /* RGB565 output */
#define OV7670_COM15_RGB555    0x30    /* RGB555 output */
#define OV7670_REG_COM16       0x41    /* Control 16 */
#define OV7670_COM16_AWBGAIN   0x08    /* AWB gain enable */
#define OV7670_REG_COM17       0x42    /* Control 17 */
#define OV7670_COM17_AECWIN    0xc0    /* AEC window - must match COM4 */
#define OV7670_COM17_CBAR      0x08    /* DSP Color bar */
#define OV7670_REG_CMATRIX_BASE 0x4f
#define OV7670_CMATRIX_LEN      6
#define OV7670_REG_CMATRIX_SIGN 0x58
#define OV7670_REG_BRIGHT      0x55    /* Brightness */
#define OV7670_REG_CONTRAS     0x56    /* Contrast control */
#define OV7670_REG_GFIX        0x69    /* Fix gain control */
#define OV7670_REG_SCALING_XSC 0x70
#define OV7670_REG_SCALING_YSC 0x71
#define OV7670_REG_REG76       0x76    /* OV's name */
#define OV7670_R76_BLKPCOR     0x80    /* Black pixel correction enable */
#define OV7670_R76_WHTPCOR     0x40    /* White pixel correction enable */
#define OV7670_REG_RGB444      0x8c    /* RGB 444 control */
#define OV7670_R444_ENABLE     0x02    /* Turn on RGB444, overrides 5x5 */
#define OV7670_R444_RGBX       0x01    /* Empty nibble at end */
#define OV7670_REG_HAECC1      0x9f    /* Hist AEC/AGC control 1 */
#define OV7670_REG_HAECC2      0xa0    /* Hist AEC/AGC control 2 */
#define OV7670_REG_BD50MAX     0xa5    /* 50hz banding step limit */
#define OV7670_REG_HAECC3      0xa6    /* Hist AEC/AGC control 3 */
#define OV7670_REG_HAECC4      0xa7    /* Hist AEC/AGC control 4 */
#define OV7670_REG_HAECC5      0xa8    /* Hist AEC/AGC control 5 */
#define OV7670_REG_HAECC6      0xa9    /* Hist AEC/AGC control 6 */
#define OV7670_REG_HAECC7      0xaa    /* Hist AEC/AGC control 7 */
#define OV7670_REG_BD60MAX     0xab    /* 60hz banding step limit */
#define OV7670_REG_SCALINGDCW  0x72    /* Scaling DCWCTR */
#define OV7670_REG_SCALINGPCLK 0x73    /* Scaling PCLK divider */

#define ILI9341_TFTWIDTH   240
#define ILI9341_TFTHEIGHT  320

#define ILI9341_NOP        0x00
#define ILI9341_SWRESET    0x01
#define ILI9341_RDDID      0x04
#define ILI9341_RDDST      0x09

#define ILI9341_SLPIN      0x10
#define ILI9341_SLPOUT     0x11
#define ILI9341_PTLON      0x12
#define ILI9341_NORON      0x13

#define ILI9341_RDMODE     0x0A
#define ILI9341_RDMADCTL   0x0B
#define ILI9341_RDPIXFMT   0x0C
#define ILI9341_RDIMGFMT   0x0D
#define ILI9341_RDSELFDIAG 0x0F

#define ILI9341_INVOFF     0x20
#define ILI9341_INVON      0x21
#define ILI9341_GAMMASET   0x26
#define ILI9341_DISPOFF    0x28
#define ILI9341_DISPON     0x29

#define ILI9341_CASET      0x2A
#define ILI9341_PASET      0x2B
#define ILI9341_RAMWR      0x2C
#define ILI9341_RAMRD      0x2E

#define ILI9341_PTLAR      0x30
#define ILI9341_MADCTL     0x36
#define ILI9341_VSCRSADD   0x37
#define ILI9341_PIXFMT     0x3A

#define ILI9341_FRMCTR1    0xB1
#define ILI9341_FRMCTR2    0xB2
#define ILI9341_FRMCTR3    0xB3
#define ILI9341_INVCTR     0xB4
#define ILI9341_DFUNCTR    0xB6

#define ILI9341_PWCTR1     0xC0
#define ILI9341_PWCTR2     0xC1
#define ILI9341_PWCTR3     0xC2
#define ILI9341_PWCTR4     0xC3
#define ILI9341_PWCTR5     0xC4
#define ILI9341_VMCTR1     0xC5
#define ILI9341_VMCTR2     0xC7

#define ILI9341_RDID1      0xDA
#define ILI9341_RDID2      0xDB
#define ILI9341_RDID3      0xDC
#define ILI9341_RDID4      0xDD

#define ILI9341_GMCTRP1    0xE0
#define ILI9341_GMCTRN1    0xE1

class ov7670{
public:
    ov7670() {}
    ~ov7670() {}

    bool begin(uint8_t i2c_addr=OV7670_ADDR){
        Wire.begin();
        _i2c_addr = i2c_addr;

        writeRegister(OV7670_REG_COM7, OV7670_COM7_RESET);
        delay(200);

        uint8_t pid = readRegister(OV7670_REG_PID);
        uint8_t ver = readRegister(OV7670_REG_VER);

        if(pid != 0x76){
            return false;
        }
        if(ver != 0x73){
            return false;
        }

        //write a configuration
        for(int i=0;i<CHANGE_REG_NUM;i++)
            writeRegister(change_reg[i][0], change_reg[i][1]);

        delay(300);

        return true;
    }

    void sleep(bool state){
        if(state)
            writeRegister(OV7670_REG_COM2, OV7670_COM2_SSLEEP);
        else
            writeRegister(OV7670_REG_COM2, 0x00);
    }

    uint8_t readRegister(uint8_t reg){
        Wire.beginTransmission(_i2c_addr);
        Wire.write(reg);                       // Register
        Wire.endTransmission();

        Wire.requestFrom(_i2c_addr, (uint8_t)1);
        return Wire.read();
    }

    void writeRegister(uint8_t reg, uint8_t value)
    {
        Wire.beginTransmission(_i2c_addr);
        Wire.write(reg);                       // Register
        Wire.write(value);
        Wire.endTransmission();
    }
private:
    uint8_t _i2c_addr;
};


#endif /* OV7670_H_ */
