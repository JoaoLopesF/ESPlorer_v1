#include "OV7670.h"
#include "XClk.h"
#include "Log.h"

OV7670::OV7670(Mode m, const int SIOD, const int SIOC, const int VSYNC, const int HREF, const int XCLK, const int PCLK, const int D0, const int D1, const int D2, const int D3, const int D4, const int D5, const int D6, const int D7)
  :i2c(SIOD, SIOC)
{
  ClockEnable(XCLK, 20000000); //base is 80MHz

  DEBUG_PRINT("Waiting for VSYNC...");
  pinMode(VSYNC, INPUT);
  while(!digitalRead(VSYNC)) yield();   ///// Put yield here - Joao Lopes
  while(digitalRead(VSYNC)) yield();
  DEBUG_PRINTLN(" done");

  mode = m;
  switch(mode)
  {

//    case VGA_RGB565:
//      blockSlice = 60;
//      xres = 640;
//      yres = 480;
//      VGARGB565();
//
//    break;
//
//    case QVGA_RGB565:
//      blockSlice = 120;
//      xres = 320;
//      yres = 240;
//      QVGARGB565();
//    break;
//
    case QQVGA_RGB565:
      blockSlice = 120;
      xres = 160;
      yres = 120;
      QQVGARGB565();
    break;

//    case QQQVGA_RGB565:
//      blockSlice = 60;
//      xres = 80;
//      yres = 60;
//      QQQVGARGB565();
//    break;

    default:
    xres = 0;
    yres = 0;
  }
  //testImage();
  I2SCamera::init(xres, yres, VSYNC, HREF, XCLK, PCLK, D0, D1, D2, D3, D4, D5, D6, D7);
}

void OV7670::testImage()
{
  i2c.writeRegister(ADDR, 0x71, 0x35 | 0x80);
}

void OV7670::saturation(int s)  //-2 to 2
{
  //color matrix values
  i2c.writeRegister(ADDR, 0x4f, 0x80 + 0x20 * s);
  i2c.writeRegister(ADDR, 0x50, 0x80 + 0x20 * s);
  i2c.writeRegister(ADDR, 0x51, 0x00);
  i2c.writeRegister(ADDR, 0x52, 0x22 + (0x11 * s) / 2);
  i2c.writeRegister(ADDR, 0x53, 0x5e + (0x2f * s) / 2);
  i2c.writeRegister(ADDR, 0x54, 0x80 + 0x20 * s);
  i2c.writeRegister(ADDR, 0x58, 0x9e);  //matrix signs
}

void OV7670::frameControl(int hStart, int hStop, int vStart, int vStop)
{
  i2c.writeRegister(ADDR, REG_HSTART, hStart >> 3);
  i2c.writeRegister(ADDR, REG_HSTOP,  hStop >> 3);
  i2c.writeRegister(ADDR, REG_HREF, ((hStop & 0b111) << 3) | (hStart & 0b111));

  i2c.writeRegister(ADDR, REG_VSTART, vStart >> 2);
  i2c.writeRegister(ADDR, REG_VSTOP, vStop >> 2);
  i2c.writeRegister(ADDR, REG_VREF, ((vStop & 0b11) << 2) | (vStart & 0b11));
}

///////////////////////////////////////


//void OV7670::QVGA()
//{
//    i2c.writeRegister(ADDR, REG_COM3, 0x04);  //DCW enable -- done
//    i2c.writeRegister(ADDR, REG_COM14, 0x19); //pixel clock divided by 4, manual scaling enable, DCW and PCLK controlled by register  -- done
//    i2c.writeRegister(ADDR, REG_SCALING_XSC, 0x3a); // -- done
//    i2c.writeRegister(ADDR, REG_SCALING_YSC, 0x35); // -- done
//    i2c.writeRegister(ADDR, REG_SCALING_DCWCTR, 0x11); //downsample by 2
//    i2c.writeRegister(ADDR, REG_SCALING_PCLK_DIV, 0xf1); //pixel clock divided by 8
//    i2c.writeRegister(ADDR, REG_SCALING_PCLK_DELAY, 0x02);
//}
//
//void OV7670::VGA()
//{
//    i2c.writeRegister(ADDR, REG_COM3, 0x0);  //DCW enable -- done
////    i2c.writeRegister(ADDR, REG_COM14, 0x19); //pixel clock divided by 4, manual scaling enable, DCW and PCLK controlled by register  -- done
//    i2c.writeRegister(ADDR, REG_COM14, 0); //pixel clock divided by 4, manual scaling enable, DCW and PCLK controlled by register  -- done
////    i2c.writeRegister(ADDR, REG_SCALING_XSC, 0x3a); // -- done
////    i2c.writeRegister(ADDR, REG_SCALING_YSC, 0x35); // -- done
//  //  i2c.writeRegister(ADDR, REG_SCALING_DCWCTR, 0x11); //downsample by 2
//  //  i2c.writeRegister(ADDR, REG_SCALING_PCLK_DIV, 0xf0); //pixel clock divided by 8
//  //  i2c.writeRegister(ADDR, REG_SCALING_PCLK_DELAY, 0x02);
//}
//
//
//void OV7670::VGARGB565()
//{
//  i2c.writeRegister(ADDR, REG_COM7, 0b10000000);  //all registers default
//
//  i2c.writeRegister(ADDR, REG_CLKRC, 0b10000000); //double clock
//  i2c.writeRegister(ADDR, REG_COM11, 0b1000 | 0b10); //enable auto 50/60Hz detect + exposure timing can be less...
//
//  i2c.writeRegister(ADDR, REG_COM7, 0b100); //RGB
//  i2c.writeRegister(ADDR, REG_COM15, 0b11000000 | 0b010000); //RGB565
//
//  /*
//  i2c.writeRegister(ADDR, REG_COM7, 0b10000000);  //all registers default
//
//  i2c.writeRegister(ADDR, REG_CLKRC, 0b10000000); //double clock
//  i2c.writeRegister(ADDR, REG_COM11, 0b1000 | 0b10); //enable auto 50/60Hz detect + exposure timing can be less...
//
//  i2c.writeRegister(ADDR, REG_COM7, 0b100); //RGB
//  i2c.writeRegister(ADDR, REG_COM15, 0b11000000 | 0b010000); //RGB565
//*/
//  VGA();
//
//  // hstart, hstop, vstart, vstop
//  frameControl(168, 24, 12, 492); //no clue why horizontal needs such strange values, vertical works ok
//
//  //i2c.writeRegister(ADDR, REG_COM10, 0x02); //VSYNC negative
//  //i2c.writeRegister(ADDR, REG_MVFP, 0x2b);  //mirror flip
//
//  i2c.writeRegister(ADDR, 0xb0, 0x84);// no clue what this is but it's most important for colors
//  saturation(0);
//  i2c.writeRegister(ADDR, 0x13, 0xe7); //AWB on
//  i2c.writeRegister(ADDR, 0x6f, 0x9f); // Simple AWB
//}
//
//
//void OV7670::QVGARGB565()
//{
//  i2c.writeRegister(ADDR, REG_COM7, 0b10000000);  //all registers default
//
//  i2c.writeRegister(ADDR, REG_CLKRC, 0b10000000); //double clock
//  i2c.writeRegister(ADDR, REG_COM11, 0b1000 | 0b10); //enable auto 50/60Hz detect + exposure timing can be less...
//
//  i2c.writeRegister(ADDR, REG_COM7, 0b100); //RGB
//  i2c.writeRegister(ADDR, REG_COM15, 0b11000000 | 0b010000); //RGB565
//
//  QVGA();
//
//  // hstart, hstop, vstart, vstop
//  frameControl(168, 24, 12, 492); //no clue why horizontal needs such strange values, vertical works ok
//
//  //i2c.writeRegister(ADDR, REG_COM10, 0x02); //VSYNC negative
//  //i2c.writeRegister(ADDR, REG_MVFP, 0x2b);  //mirror flip
//
//  i2c.writeRegister(ADDR, 0xb0, 0x84);// no clue what this is but it's most important for colors
//  saturation(0);
//  i2c.writeRegister(ADDR, 0x13, 0xe7); //AWB on
//  i2c.writeRegister(ADDR, 0x6f, 0x9f); // Simple AWB
//}
//

////////////////////////////////////
//void OV7670::QQQVGA()
//{
//    i2c.writeRegister(ADDR, REG_COM3, 0x04);  //DCW enable
//    i2c.writeRegister(ADDR, REG_COM14, 0x1b); //pixel clock divided by 4, manual scaling enable, DCW and PCLK controlled by register
//    i2c.writeRegister(ADDR, REG_SCALING_XSC, 0x3a);
//    i2c.writeRegister(ADDR, REG_SCALING_YSC, 0x35);
//    i2c.writeRegister(ADDR, REG_SCALING_DCWCTR, 0x33); //downsample by 8
//    i2c.writeRegister(ADDR, REG_SCALING_PCLK_DIV, 0xf3); //pixel clock divided by 8
//    i2c.writeRegister(ADDR, REG_SCALING_PCLK_DELAY, 0x02);
//}

void OV7670::QQVGA()
{
  //160x120 (1/4)
  //i2c.writeRegister(ADDR, REG_CLKRC, 0x01);
  i2c.writeRegister(ADDR, REG_COM3, 0x04);  //DCW enable

  i2c.writeRegister(ADDR, REG_COM14, 0x1a); //pixel clock divided by 4, manual scaling enable, DCW and PCLK controlled by register
  i2c.writeRegister(ADDR, REG_SCALING_XSC, 0x3a);
  i2c.writeRegister(ADDR, REG_SCALING_YSC, 0x35);

  i2c.writeRegister(ADDR, REG_SCALING_DCWCTR, 0x22); //downsample by 4
  i2c.writeRegister(ADDR, REG_SCALING_PCLK_DIV, 0xf2); //pixel clock divided by 4
  i2c.writeRegister(ADDR, REG_SCALING_PCLK_DELAY, 0x02);
}

void OV7670::QQVGARGB565()
{

  i2c.writeRegister(ADDR, REG_COM7, 0b10000000);  //all registers default

  i2c.writeRegister(ADDR, REG_CLKRC, 0b10000000); //double clock
//JL i2c.writeRegister(ADDR, REG_CLKRC, 0b10000000 | 0b00000001); // 30 fps clock

  i2c.writeRegister(ADDR, REG_COM11, 0b1000 | 0b10); //enable auto 50/60Hz detect + exposure timing can be less...
//JL  i2c.writeRegister(ADDR, REG_COM11, 0b10000000 | 0b1000 | 0b10); // Night mode ??
//JL  i2c.writeRegister(ADDR, REG_COM11, 0b10); // Only bit 1
  i2c.writeRegister(ADDR, REG_COM7, 0b100); //RGB
  i2c.writeRegister(ADDR, REG_COM15, 0b11000000 | 0b010000); //RGB565

  QQVGA();

  frameControl(196, 52, 8, 488); //no clue why horizontal needs such strange values, vertical works ok

  //i2c.writeRegister(ADDR, REG_COM10, 0x02); //VSYNC negative
  //i2c.writeRegister(ADDR, REG_MVFP, 0x2b);  //mirror flip

  i2c.writeRegister(ADDR, REG_MVFP, 0b00000001 | 0b00000010);  // 0x01 & black sun

  i2c.writeRegister(ADDR, 0xb0, 0x84);// no clue what this is but it's most important for colors

  saturation(0);

  i2c.writeRegister(ADDR, 0x13, 0xe7); //AWB on
  i2c.writeRegister(ADDR, 0x6f, 0x9f); // Simple AWB

  ///// Begin changes - Joao Lopes
#ifdef REGISTERS_LINUX
  writeRegistersLinux();
#endif
  /////// End changes - Joao Lopes

}

//void OV7670::QQQVGARGB565()
//{
//  i2c.writeRegister(ADDR, REG_COM7, 0b10000000);  //all registers default
//
//  i2c.writeRegister(ADDR, REG_CLKRC, 0b10000000); //double clock
//  i2c.writeRegister(ADDR, REG_COM11, 0b1000 | 0b10); //enable auto 50/60Hz detect + exposure timing can be less...
//
//  i2c.writeRegister(ADDR, REG_COM7, 0b100); //RGB
//  i2c.writeRegister(ADDR, REG_COM15, 0b11000000 | 0b010000); //RGB565
//
//  QQQVGA();
//
//  frameControl(196, 52, 8, 488); //no clue why horizontal needs such strange values, vertical works ok
//
//  //i2c.writeRegister(ADDR, REG_MVFP, 0x2b);  //mirror flip
//
//  i2c.writeRegister(ADDR, 0xb0, 0x84);// no clue what this is but it's most important for colors
//  saturation(0);
//  i2c.writeRegister(ADDR, 0x13, 0xe7); //AWB on
//  i2c.writeRegister(ADDR, 0x6f, 0x9f); // Simple AWB
//}

///// Begin changes - Joao Lopes

#ifdef REGISTERS_LINUX

// Write registers based from linux driver: https://github.com/torvalds/linux/blob/master/drivers/media/i2c/ov7670.c

struct regval_list{
  uint8_t reg_num;
  uint8_t value;
};


void OV7670::writeRegistersLinux () {

	const struct regval_list ov7670_default_regs[] = { // from the linux driver
//		{ REG_CLKRC, 0x1 },	/* OV: clock scale (30 fps) */
//		{ REG_TSLB,  0x04 },	/* OV */
//			{ REG_COM7, 0 },	/* VGA */
//			/*
//			 * Set the hardware window.  These values from OV don't entirely
//			 * make sense - hstop is less than hstart.  But they work...
//			 */
//			{ REG_HSTART, 0x13 },	{ REG_HSTOP, 0x01 },
//			{ REG_HREF, 0xb6 },	{ REG_VSTART, 0x02 },
//			{ REG_VSTOP, 0x7a },	{ REG_VREF, 0x0a },
//
//			{ REG_COM3, 0 },	{ REG_COM14, 0 },
//			/* Mystery scaling numbers */
//			{ REG_SCALING_XSC, 0x3a },
//			{ REG_SCALING_YSC, 0x35 },
//			{ 0x72, 0x11 },		{ 0x73, 0xf0 },
//			{ 0xa2, 0x02 },		{ REG_COM10, 0x0 },

		/* Gamma curve values */
		{ 0x7a, 0x20 },		{ 0x7b, 0x10 },
		{ 0x7c, 0x1e },		{ 0x7d, 0x35 },
		{ 0x7e, 0x5a },		{ 0x7f, 0x69 },
		{ 0x80, 0x76 },		{ 0x81, 0x80 },
		{ 0x82, 0x88 },		{ 0x83, 0x8f },
		{ 0x84, 0x96 },		{ 0x85, 0xa3 },
		{ 0x86, 0xaf },		{ 0x87, 0xc4 },
		{ 0x88, 0xd7 },		{ 0x89, 0xe8 },

		/* AGC and AEC parameters.  Note we start by disabling those features,
		   then turn them only after tweaking the values. */
		{ REG_COM8, COM8_FASTAEC | COM8_AECSTEP | COM8_BFILT },
		{ REG_GAIN, 0 },	{ REG_AECH, 0 },
		{ REG_COM4, 0x40 }, /* magic reserved bit */
		{ REG_COM9, 0x18 }, /* 4x gain + magic rsvd bit */
		{ REG_BD50MAX, 0x05 },	{ REG_BD60MAX, 0x07 },
		{ REG_AEW, 0x95 },	{ REG_AEB, 0x33 },
		{ REG_VPT, 0xe3 },	{ REG_HAECC1, 0x78 },
		{ REG_HAECC2, 0x68 },	{ 0xa1, 0x03 }, /* magic */
		{ REG_HAECC3, 0xd8 },	{ REG_HAECC4, 0xd8 },
		{ REG_HAECC5, 0xf0 },	{ REG_HAECC6, 0x90 },
		{ REG_HAECC7, 0x94 },
		{ REG_COM8, COM8_FASTAEC|COM8_AECSTEP|COM8_BFILT|COM8_AGC|COM8_AEC },
//
//		/* Almost all of these are magic "reserved" values.  */
//		{ REG_COM5, 0x61 },	{ REG_COM6, 0x4b },
//		{ 0x16, 0x02 },		{ REG_MVFP, 0x07 },
//		{ 0x21, 0x02 },		{ 0x22, 0x91 },
//		{ 0x29, 0x07 },		{ 0x33, 0x0b },
//		{ 0x35, 0x0b },		{ 0x37, 0x1d },
//		{ 0x38, 0x71 },		{ 0x39, 0x2a },
//		{ REG_COM12, 0x78 },	{ 0x4d, 0x40 },
//		{ 0x4e, 0x20 },		{ REG_GFIX, 0 },
//		{ 0x6b, 0x4a },		{ 0x74, 0x10 },
//		{ 0x8d, 0x4f },		{ 0x8e, 0 },
//		{ 0x8f, 0 },		{ 0x90, 0 },
//		{ 0x91, 0 },		{ 0x96, 0 },
//		{ 0x9a, 0 },		{ 0xb0, 0x84 },
//		{ 0xb1, 0x0c },		{ 0xb2, 0x0e },
//		{ 0xb3, 0x82 },		{ 0xb8, 0x0a },
//
		/* More reserved magic, some of which tweaks white balance */
		{ 0x43, 0x0a },		{ 0x44, 0xf0 },
		{ 0x45, 0x34 },		{ 0x46, 0x58 },
		{ 0x47, 0x28 },		{ 0x48, 0x3a },
		{ 0x59, 0x88 },		{ 0x5a, 0x88 },
		{ 0x5b, 0x44 },		{ 0x5c, 0x67 },
		{ 0x5d, 0x49 },		{ 0x5e, 0x0e },
		{ 0x6c, 0x0a },		{ 0x6d, 0x55 },
		{ 0x6e, 0x11 },		{ 0x6f, 0x9f }, /* "9e for advance AWB" */
		{ 0x6a, 0x40 },		{ REG_BLUE, 0x40 },
		{ REG_RED, 0x60 },
		{ REG_COM8, COM8_FASTAEC|COM8_AECSTEP|COM8_BFILT|COM8_AGC|COM8_AEC|COM8_AWB },

		/* Matrix coefficients */
		{ 0x4f, 0x80 },		{ 0x50, 0x80 },
		{ 0x51, 0 },		{ 0x52, 0x22 },
		{ 0x53, 0x5e },		{ 0x54, 0x80 },
		{ 0x58, 0x9e },

		{ REG_COM16, COM16_AWBGAIN },	{ REG_EDGE, 0 },
		{ 0x75, 0x05 },		{ 0x76, 0xe1 },
		{ 0x4c, 0 },		{ 0x77, 0x01 },
		{ REG_COM13, 0xc3 },	{ 0x4b, 0x09 },
		{ 0xc9, 0x60 },		{ REG_COM16, 0x38 },
		{ 0x56, 0x40 },

//		{ 0x34, 0x11 },		{ REG_COM11, COM11_EXP|COM11_HZAUTO },
		{ 0x34, 0x11 },		{ REG_COM11, COM11_EXP},
		{ 0xa4, 0x88 },		{ 0x96, 0 },
		{ 0x97, 0x30 },		{ 0x98, 0x20 },
		{ 0x99, 0x30 },		{ 0x9a, 0x84 },
		{ 0x9b, 0x29 },		{ 0x9c, 0x03 },
		{ 0x9d, 0x4c },		{ 0x9e, 0x3f },
		{ 0x78, 0x04 },

//		/* Extra-weird stuff.  Some sort of multiplexor register */
//		{ 0x79, 0x01 },		{ 0xc8, 0xf0 },
//		{ 0x79, 0x0f },		{ 0xc8, 0x00 },
//		{ 0x79, 0x10 },		{ 0xc8, 0x7e },
//		{ 0x79, 0x0a },		{ 0xc8, 0x80 },
//		{ 0x79, 0x0b },		{ 0xc8, 0x01 },
//		{ 0x79, 0x0c },		{ 0xc8, 0x0f },
//		{ 0x79, 0x0d },		{ 0xc8, 0x20 },
//		{ 0x79, 0x09 },		{ 0xc8, 0x80 },
//		{ 0x79, 0x02 },		{ 0xc8, 0xc0 },
//		{ 0x79, 0x03 },		{ 0xc8, 0x40 },
//		{ 0x79, 0x05 },		{ 0xc8, 0x30 },
//		{ 0x79, 0x26 },
//
		{ 0xff, 0xff },	/* END MARKER */
	};

	// Write registers from linux driver

	uint8_t reg_addr, reg_val;
	for (uint8_t i=0;; i++) {
		reg_addr = ov7670_default_regs[i].reg_num;
		reg_val = ov7670_default_regs[i].value;
		if (reg_addr == 0xff || reg_val == 0xff) {
			break;
		}
		i2c.writeRegister(ADDR, reg_addr, reg_val);
	}

}

#endif // REGISTERS_LINUX

/////// End changes - Joao Lopes



