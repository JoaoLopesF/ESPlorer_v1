#pragma once
#include "I2SCamera.h"
#include "I2C.h"

///// Begin changes - Joao Lopes

// Write registers based on linux driver ?

//#define REGISTERS_LINUX true

#ifdef REGISTERS_LINUX

/* Registers * based on linux driver
 */

/* Registers */
#define REG_GAIN	0x00	/* Gain lower 8 bits (rest in vref) */
#define REG_BLUE	0x01	/* blue gain */
#define REG_RED		0x02	/* red gain */
#define REG_VREF	0x03	/* Pieces of GAIN, VSTART, VSTOP */
#define REG_COM1	0x04	/* Control 1 */
#define  COM1_CCIR656	  0x40  /* CCIR656 enable */
#define REG_BAVE	0x05	/* U/B Average level */
#define REG_GbAVE	0x06	/* Y/Gb Average level */
#define REG_AECHH	0x07	/* AEC MS 5 bits */
#define REG_RAVE	0x08	/* V/R Average level */
#define REG_COM2	0x09	/* Control 2 */
#define  COM2_SSLEEP	  0x10	/* Soft sleep mode */
#define REG_PID		0x0a	/* Product ID MSB */
#define REG_VER		0x0b	/* Product ID LSB */
#define REG_COM3	0x0c	/* Control 3 */
#define  COM3_SWAP	  0x40	  /* Byte swap */
#define  COM3_SCALEEN	  0x08	  /* Enable scaling */
#define  COM3_DCWEN	  0x04	  /* Enable downsamp/crop/window */
#define REG_COM4	0x0d	/* Control 4 */
#define REG_COM5	0x0e	/* All "reserved" */
#define REG_COM6	0x0f	/* Control 6 */
#define REG_AECH	0x10	/* More bits of AEC value */
#define REG_CLKRC	0x11	/* Clocl control */
#define   CLK_EXT	  0x40	  /* Use external clock directly */
#define   CLK_SCALE	  0x3f	  /* Mask for internal clock scale */
#define REG_COM7	0x12	/* Control 7 */
#define   COM7_RESET	  0x80	  /* Register reset */
#define   COM7_FMT_MASK	  0x38
#define   COM7_FMT_VGA	  0x00
#define	  COM7_FMT_CIF	  0x20	  /* CIF format */
#define   COM7_FMT_QVGA	  0x10	  /* QVGA format */
#define   COM7_FMT_QCIF	  0x08	  /* QCIF format */
#define	  COM7_RGB	  0x04	  /* bits 0 and 2 - RGB format */
#define	  COM7_YUV	  0x00	  /* YUV */
#define	  COM7_BAYER	  0x01	  /* Bayer format */
#define	  COM7_PBAYER	  0x05	  /* "Processed bayer" */
#define REG_COM8	0x13	/* Control 8 */
#define   COM8_FASTAEC	  0x80	  /* Enable fast AGC/AEC */
#define   COM8_AECSTEP	  0x40	  /* Unlimited AEC step size */
#define   COM8_BFILT	  0x20	  /* Band filter enable */
#define   COM8_AGC	  0x04	  /* Auto gain enable */
#define   COM8_AWB	  0x02	  /* White balance enable */
#define   COM8_AEC	  0x01	  /* Auto exposure enable */
#define REG_COM9	0x14	/* Control 9  - gain ceiling */
#define REG_COM10	0x15	/* Control 10 */
#define   COM10_HSYNC	  0x40	  /* HSYNC instead of HREF */
#define   COM10_PCLK_HB	  0x20	  /* Suppress PCLK on horiz blank */
#define   COM10_HREF_REV  0x08	  /* Reverse HREF */
#define   COM10_VS_LEAD	  0x04	  /* VSYNC on clock leading edge */
#define   COM10_VS_NEG	  0x02	  /* VSYNC negative */
#define   COM10_HS_NEG	  0x01	  /* HSYNC negative */
#define REG_HSTART	0x17	/* Horiz start high bits */
#define REG_HSTOP	0x18	/* Horiz stop high bits */
#define REG_VSTART	0x19	/* Vert start high bits */
#define REG_VSTOP	0x1a	/* Vert stop high bits */
#define REG_PSHFT	0x1b	/* Pixel delay after HREF */
#define REG_MIDH	0x1c	/* Manuf. ID high */
#define REG_MIDL	0x1d	/* Manuf. ID low */
#define REG_MVFP	0x1e	/* Mirror / vflip */
#define   MVFP_MIRROR	  0x20	  /* Mirror image */
#define   MVFP_FLIP	  0x10	  /* Vertical flip */

#define REG_AEW		0x24	/* AGC upper limit */
#define REG_AEB		0x25	/* AGC lower limit */
#define REG_VPT		0x26	/* AGC/AEC fast mode op region */
#define REG_HSYST	0x30	/* HSYNC rising edge delay */
#define REG_HSYEN	0x31	/* HSYNC falling edge delay */
#define REG_HREF	0x32	/* HREF pieces */
#define REG_TSLB	0x3a	/* lots of stuff */
#define   TSLB_YLAST	  0x04	  /* UYVY or VYUY - see com13 */
#define REG_COM11	0x3b	/* Control 11 */
#define   COM11_NIGHT	  0x80	  /* NIght mode enable */
#define   COM11_NMFR	  0x60	  /* Two bit NM frame rate */
#define   COM11_HZAUTO	  0x10	  /* Auto detect 50/60 Hz */
#define	  COM11_50HZ	  0x08	  /* Manual 50Hz select */
#define   COM11_EXP	  0x02
#define REG_COM12	0x3c	/* Control 12 */
#define   COM12_HREF	  0x80	  /* HREF always */
#define REG_COM13	0x3d	/* Control 13 */
#define   COM13_GAMMA	  0x80	  /* Gamma enable */
#define	  COM13_UVSAT	  0x40	  /* UV saturation auto adjustment */
#define   COM13_UVSWAP	  0x01	  /* V before U - w/TSLB */
#define REG_COM14	0x3e	/* Control 14 */
#define   COM14_DCWEN	  0x10	  /* DCW/PCLK-scale enable */
#define REG_EDGE	0x3f	/* Edge enhancement factor */
#define REG_COM15	0x40	/* Control 15 */
#define   COM15_R10F0	  0x00	  /* Data range 10 to F0 */
#define	  COM15_R01FE	  0x80	  /*            01 to FE */
#define   COM15_R00FF	  0xc0	  /*            00 to FF */
#define   COM15_RGB565	  0x10	  /* RGB565 output */
#define   COM15_RGB555	  0x30	  /* RGB555 output */
#define REG_COM16	0x41	/* Control 16 */
#define   COM16_AWBGAIN   0x08	  /* AWB gain enable */
#define REG_COM17	0x42	/* Control 17 */
#define   COM17_AECWIN	  0xc0	  /* AEC window - must match COM4 */
#define   COM17_CBAR	  0x08	  /* DSP Color bar */

/*
 * This matrix defines how the colors are generated, must be
 * tweaked to adjust hue and saturation.
 *
 * Order: v-red, v-green, v-blue, u-red, u-green, u-blue
 *
 * They are nine-bit signed quantities, with the sign bit
 * stored in 0x58.  Sign for v-red is bit 0, and up from there.
 */
#define	REG_CMATRIX_BASE 0x4f
#define   CMATRIX_LEN 6
#define REG_CMATRIX_SIGN 0x58


#define REG_BRIGHT	0x55	/* Brightness */
#define REG_CONTRAS	0x56	/* Contrast control */

#define REG_GFIX	0x69	/* Fix gain control */

#define REG_DBLV	0x6b	/* PLL control an debugging */
#define   DBLV_BYPASS	  0x00	  /* Bypass PLL */
#define   DBLV_X4	  0x01	  /* clock x4 */
#define   DBLV_X6	  0x10	  /* clock x6 */
#define   DBLV_X8	  0x11	  /* clock x8 */

#define REG_SCALING_XSC	0x70	/* Test pattern and horizontal scale factor */
#define   TEST_PATTTERN_0 0x80
#define REG_SCALING_YSC	0x71	/* Test pattern and vertical scale factor */
#define   TEST_PATTTERN_1 0x80

#define REG_REG76	0x76	/* OV's name */
#define   R76_BLKPCOR	  0x80	  /* Black pixel correction enable */
#define   R76_WHTPCOR	  0x40	  /* White pixel correction enable */

#define REG_RGB444	0x8c	/* RGB 444 control */
#define   R444_ENABLE	  0x02	  /* Turn on RGB444, overrides 5x5 */
#define   R444_RGBX	  0x01	  /* Empty nibble at end */

#define REG_HAECC1	0x9f	/* Hist AEC/AGC control 1 */
#define REG_HAECC2	0xa0	/* Hist AEC/AGC control 2 */

#define REG_BD50MAX	0xa5	/* 50hz banding step limit */
#define REG_HAECC3	0xa6	/* Hist AEC/AGC control 3 */
#define REG_HAECC4	0xa7	/* Hist AEC/AGC control 4 */
#define REG_HAECC5	0xa8	/* Hist AEC/AGC control 5 */
#define REG_HAECC6	0xa9	/* Hist AEC/AGC control 6 */
#define REG_HAECC7	0xaa	/* Hist AEC/AGC control 7 */
#define REG_BD60MAX	0xab	/* 60hz banding step limit */

///// Registers that not present in linux driver

#define REG_SCALING_DCWCTR 		0x72
#define REG_SCALING_PCLK_DIV 	0x73
#define REG_SCALING_PCLK_DELAY 	0xa2

#endif

///// End changes - Joao Lopes

class OV7670: public I2SCamera
{
  public:
  enum Mode
  {
    QQQVGA_RGB565,
    QQVGA_RGB565,
    QVGA_RGB565,
    VGA_RGB565,
  };
  int xres, yres;

  protected:
  static const int ADDR = 0x42;
  
  Mode mode;
  I2C i2c;

  void testImage();
  void saturation(int s);
  void frameControl(int hStart, int hStop, int vStart, int vStop);
  void QVGA();
  void QVGARGB565();
  void VGA();
  void VGARGB565();

  void QQVGA();
  void QQVGARGB565();
  void QQQVGA();
  void QQQVGARGB565();
  void inline writeRegister(unsigned char reg, unsigned char data)
  {
    i2c.writeRegister(ADDR, reg, data);
  }

  ///// Begin changes - Joao Lopes
#ifdef REGISTERS_LINUX
  void writeRegistersLinux();
#endif
  ///// End changes - Joao Lopes


  public:
  OV7670(OV7670::Mode m, const int SIOD, const int SIOC, const int VSYNC, const int HREF, const int XCLK, const int PCLK, const int D0, const int D1, const int D2, const int D3, const int D4, const int D5, const int D6, const int D7);

 ///// Begin changes - Joao Lopes

#ifndef REGISTERS_LINUX

//camera registers
  static const int REG_GAIN = 0x00;
  static const int REG_BLUE = 0x01;
  static const int REG_RED = 0x02;
  static const int REG_COM1 = 0x04;
  static const int REG_VREF = 0x03;
  static const int REG_COM4 = 0x0d;
  static const int REG_COM5 = 0x0e;
  static const int REG_COM6 = 0x0f;
  static const int REG_AECH = 0x10;
  static const int REG_CLKRC = 0x11;
  static const int REG_COM7 = 0x12;
    static const int COM7_RGB = 0x04;
  static const int REG_COM8 = 0x13;
    static const int COM8_FASTAEC = 0x80;    // Enable fast AGC/AEC
    static const int COM8_AECSTEP = 0x40;    // Unlimited AEC step size
    static const int COM8_BFILT = 0x20;    // Band filter enable
    static const int COM8_AGC = 0x04;    // Auto gain enable
    static const int COM8_AWB = 0x02;    // White balance enable
    static const int COM8_AEC = 0x0;
  static const int REG_COM9 = 0x14;
  static const int REG_COM10 = 0x15;
  static const int REG_COM14 = 0x3E;
  static const int REG_COM11 = 0x3B;
  static const int COM11_NIGHT = 0x80;
  static const int COM11_NMFR = 0x60;
  static const int COM11_HZAUTO = 0x10;
  static const int COM11_50HZ = 0x08;
  static const int COM11_EXP = 0x0;
  static const int REG_TSLB = 0x3A;
  static const int REG_RGB444 = 0x8C;
  static const int REG_COM15 = 0x40;
    static const int COM15_RGB565 = 0x10;
    static const int COM15_R00FF = 0xc0;
  static const int REG_HSTART = 0x17;
  static const int REG_HSTOP = 0x18;
  static const int REG_HREF = 0x32;
  static const int REG_VSTART = 0x19;
  static const int REG_VSTOP = 0x1A;
  static const int REG_COM3 = 0x0C;
  static const int REG_MVFP = 0x1E;
  static const int REG_COM13 = 0x3d;
    static const int COM13_UVSAT = 0x40;
  static const int REG_SCALING_XSC = 0x70;
  static const int REG_SCALING_YSC = 0x71;    
  static const int REG_SCALING_DCWCTR = 0x72;
  static const int REG_SCALING_PCLK_DIV = 0x73;
  static const int REG_SCALING_PCLK_DELAY = 0xa2;
  static const int REG_BD50MAX = 0xa5;
  static const int REG_BD60MAX = 0xab;
  static const int REG_AEW = 0x24;
  static const int REG_AEB = 0x25;
  static const int REG_VPT = 0x26;
  static const int REG_HAECC1 = 0x9f;
  static const int REG_HAECC2 = 0xa0;
  static const int REG_HAECC3 = 0xa6;
  static const int REG_HAECC4 = 0xa7;
  static const int REG_HAECC5 = 0xa8;
  static const int REG_HAECC6 = 0xa9;
  static const int REG_HAECC7 = 0xaa;
  static const int REG_COM12 = 0x3c;
  static const int REG_GFIX = 0x69;
  static const int REG_COM16 = 0x41;
  static const int COM16_AWBGAIN = 0x08;
  static const int REG_EDGE = 0x3f;
  static const int REG_REG76 = 0x76;
  static const int ADCCTR0 = 0x20;

#endif

  ///// End changes - Joao Lopes

};

