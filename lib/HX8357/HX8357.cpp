
#include "HX8357.h"
#include "Commands.h"

extern "C" {
  #include "../espspi/espspi.h"
  #include "../espspi/spi_interface.h"
}

#define CMD 0
#define DATA 1

#define MADCTL_MY  0x80
#define MADCTL_MX  0x40
#define MADCTL_MV  0x20
#define MADCTL_ML  0x10
#define MADCTL_RGB 0x00
#define MADCTL_BGR 0x08
#define MADCTL_MH  0x04

HX8357::HX8357() : Adafruit_GFX(TFTWIDTH, TFTHEIGHT) {

}

void HX8357::begin() {

  SpiAttr hSpiAttr;
  hSpiAttr.bitOrder = SpiBitOrder_MSBFirst;
  hSpiAttr.speed = SpiSpeed_80MHz;
  hSpiAttr.mode = SpiMode_Master;
  hSpiAttr.subMode = SpiSubMode_0;


  //Init HSPI GPIO
  WRITE_PERI_REG(PERIPHS_IO_MUX, 0x105);
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDI_U, 2);//configure io to spi mode
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U, 2);//configure io to spi mode
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTMS_U, 2);//configure io to spi mode
  PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDO_U, 2);//configure io to spi mode

  SPIInit(SpiNum_HSPI, &hSpiAttr);

  //
  SET_PERI_REG_MASK(SPI_USER(ESPHSPI), SPI_USR_COMMAND|SPI_CS_HOLD);
  CLEAR_PERI_REG_MASK(SPI_USER(ESPHSPI), SPI_USR_DUMMY|SPI_USR_ADDR|SPI_USR_MOSI);
  WRITE_PERI_REG(SPI_USER1(ESPHSPI), ((8&SPI_USR_COMMAND_BITLEN)<<SPI_USR_COMMAND_BITLEN_S));

  uint8_t data[15] = {0};

  write_command(HX8357_DISPOFF);

  delay(10);  // has to be 10ms after sleep out

  write_command(HX8357_SLPOUT);

  delay(120); // has to be 120ms after display off


  data[0] = 0x02;
  data[1] = 0x01;
  data[2] = 0x02;
  data[3] = 0x01;
  write_command_data(HX8357_SETEQ, data, 4); //Set EQ function (no idea what that does)
  

  data[0] = 0x00;
  data[1] = 0x00;
  data[2] = 0x9A;
  data[3] = 0x9A;
  data[4] = 0x9B;
  data[5] = 0x9B;
  data[6] = 0x00;
  data[7] = 0x00;
  data[8] = 0x00;
  data[9] = 0x00;
  data[10] = 0xAE;
  data[11] = 0xAE;
  data[12] = 0x01;
  data[13] = 0xA2;
  data[14] = 0x00;
  write_command_data(0xED, data, 15); // Not in HX8257 spec. but was transmitted by the MPSM UI Board
  

  //
  data[0] = 0x00;
  write_command_data(HX8257_SETDISPLAY, data, 1); // DBI Interface (CPU) selected and set the display mode to internal oscillation clock
  


  data[0] = 0x10;
  write_command_data(HX8357_SETNORTIM, data, 1); // Parameter count does not match specification. Should be 3 parameters. MPSM was only sending one.
  

  //
  //
  data[0] = 0x00;
  data[1] = 0x46;
  data[2] = 0x12;
  data[3] = 0x20;
  data[4] = 0x0C;
  data[5] = 0x00;
  data[6] = 0x56;
  data[7] = 0x12;
  data[8] = 0x67;
  data[9] = 0x02;
  data[10] = 0x00;
  data[11] = 0x0C;
  write_command_data(HX8357_SETGAMMA, data, 12); // "This command is used for Gamma Curve related Setting.". I'll just keep it as the MPSM is sending it.
  

  //
  data[0] = 0x44;
  data[1] = 0x42;
  data[2] = 0x06;
  write_command_data(HX8357_SETPOWER, data, 3);
  

  //
  //
  data[0] = 0x43;
  data[1] = 0x16;
  write_command_data(HX8357_SETVCOM, data, 2);
  

  //
  data[0] = 0x04;
  data[1] = 0x22;
  write_command_data(HX8357_SETNORPOW, data, 2);
  

  //
  data[0] = 0x04;
  data[1] = 0x12;
  write_command_data(HX8357_SETPARPOW, data, 2);
  

  //
  data[0] = 0x07;
  data[1] = 0x12;
  write_command_data(HX8357_SETIDLPOW, data, 2);
  

  data[0] = 0x0C;
  write_command_data(HX8357_SETOSC, data, 1);
  

  data[0] = 0xA0;
  write_command_data(HX8357_MADCTL, data, 1); // Defines read/write scanning direction of frame memory.
  


  data[0] = 0x55;
  write_command_data(HX8357_COLMOD, data, 1); // Interface Pixel Format. Is set to 16 Bit per Pixel
  


  data[0] = 0x00;
  data[1] = 0x00;
  data[2] = 0x01; //  0x013F = 319 Decimal (full width of display)
  data[3] = 0x3F; //
  write_command_data(HX8357_CASET, data, 4); // Column address set
  

  data[0] = 0x00;
  data[1] = 0x00;
  data[2] = 0x01; //  0x01E0 = 480 Decimal (full height of display)
  data[3] = 0xE0; //
  write_command_data(HX8357_PASET, data, 4); // Page address set
  


  delay(20);


  data[0] = 0xFF;
  write_command_data(HX8357_TEON, data, 1); // Turning Tearing Effect Line OFF
  

  //data[0] = 0x00;
  //write_command_data(HX8357_TEON, data, 1); // Turning Tearing Effect Line OFF
  //

  data[0] = 0x10;
  write_command_data(HX8357_WRDISBV, data, 1); // This command is used to adjust the brightness value of the display. 0x00 = lowest, 0xFF = highest
  



  data[0] = 0x24; // 0x24 = 00100100
                  //          |  ↳ BL: Backlight Control On/Off
                  //          ↳ Brightness Control Block On/Off, This bit is always used to switch brightness for display
  write_command_data(HX8357_WRCTRLD, data, 1); // This command is used to control display brightness. Backlight
  



  data[0] = 0x00; // Off
  write_command_data(HX8357_WRCABC, data, 1); // Write Content Adaptive Brightness Control
  


  data[0] = 0x00;
  write_command_data(HX8357_WRCABCMB, data, 1); // This command is used to set the minimum brightness value of the display for CABC function.
  

  // delay(5);
  write_command(HX8357_DISPOFF);
  // delay(10);
  
  write_command(HX8357_SLPOUT);
  delay(120); // has to be 120ms after display off

  data[0] = 0x02;
  data[1] = 0x01;
  data[2] = 0x02;
  data[3] = 0x01;
  write_command_data(HX8357_SETEQ, data, 4); //Set EQ function (no idea what that does)
  


  // data[0] = 0x00;
  // data[1] = 0x00;
  // data[2] = 0x9A;
  // data[3] = 0x9A;
  // data[4] = 0x9B;
  // data[5] = 0x9B;
  // data[6] = 0x00;
  // data[7] = 0x00;
  // data[8] = 0x00;
  // data[9] = 0x00;
  // data[10] = 0xAE;
  // data[11] = 0xAE;
  // data[12] = 0x01;
  // data[13] = 0xA2;
  // data[14] = 0x00;
  // write_command_data(0xED, data, 15); // Not in HX8257 spec. but was transmitted by the MPSM UI Board
  //


  data[0] = 0x00;
  write_command_data(HX8257_SETDISPLAY, data, 1); // DBI Interface (CPU) selected and set the display mode to internal oscillation clock
  

  data[0] = 0x10;
  data[1] = 0x10;
  write_command_data(HX8357_SETNORTIM, data, 2); // Parameter count does not match specification. Should be 3 parameters. MPSM was only sending one.
  


  data[0] = 0x00;
  data[1] = 0x46;
  data[2] = 0x12;
  data[3] = 0x20;
  data[4] = 0x0C;
  data[5] = 0x00;
  data[6] = 0x56;
  data[7] = 0x12;
  data[8] = 0x67;
  data[9] = 0x02;
  data[10] = 0x00;
  data[11] = 0x0C;
  write_command_data(HX8357_SETGAMMA, data, 12); // "This command is used for Gamma Curve related Setting.". I'll just keep it as the MPSM is sending it.
  


  data[0] = 0x44;
  data[1] = 0x42;
  data[2] = 0x06;
  write_command_data(HX8357_SETPOWER, data, 3);
  


  data[0] = 0x43;
  data[1] = 0x16;
  write_command_data(HX8357_SETVCOM, data, 2);
  

  data[0] = 0x04;
  data[1] = 0x22;
  write_command_data(HX8357_SETNORPOW, data, 2);
  

  data[0] = 0x04;
  data[1] = 0x12;
  write_command_data(HX8357_SETPARPOW, data, 2);
  

  data[0] = 0x07;
  data[1] = 0x12;
  write_command_data(HX8357_SETIDLPOW, data, 2);
  


  data[0] = 0x01;
  write_command_data(HX8357_SETPANELREL, data, 1);
  

  data[0] = 0x0C;
  write_command_data(HX8357_SETOSC, data, 1);
  


  data[0] = 0x55;
  write_command_data(HX8357_COLMOD, data, 1); // Interface Pixel Format. Is set to 16 Bit per Pixel
  

  data[0] = 0x00;
  data[1] = 0x00;
  data[2] = 0x01; //  0x013F = 319 Decimal (full width of display)
  data[3] = 0x3F; //
  write_command_data(HX8357_CASET, data, 4); // Column address set
  

  data[0] = 0x00;
  data[1] = 0x00;
  data[2] = 0x01; //  0x01E0 = 480 Decimal (full height of display)
  data[3] = 0xE0; //
  write_command_data(HX8357_PASET, data, 4); // Page address set
  
  // delay(20);

  data[0] = 0x10;
  write_command_data(HX8357_TEON, data, 1); // Turning Tearing Effect Line OFF
  

  //data[0] = 0x10; // 0xFF?
  //write_command_data(HX8357_TEON, data, 1); // Turning Tearing Effect Line OFF
  //


  data[0] = 0x00;
  write_command_data(HX8357_WRDISBV, data, 1); // This command is used to adjust the brightness value of the display. 0x00 = lowest, 0xFF = highest
  


  data[0] = 0x24; // 0x24 = 00100100
                  //          |  ↳ BL: Backlight Control On/Off
                  //          ↳ Brightness Control Block On/Off, This bit is always used to switch brightness for display
  write_command_data(HX8357_WRCTRLD, data, 1); // This command is used to control display brightness. Backlight
  


  data[0] = 0x00; // Off
  write_command_data(HX8357_WRCABC, data, 1); // Write Content Adaptive Brightness Control
  


  data[0] = 0x00;
  write_command_data(HX8357_WRCABCMB, data, 1); // This command is used to set the minimum brightness value of the display for CABC function.
  

  // delay(5);


  write_command(HX8357_DISPON); // Finally turn the display on (actually the internal clock) to see the GRAM content as it gets filled. Usually you first push all data through SPI and then turn it on. But we want to see whats happening :)
  
}

void HX8357::write_command(uint8_t cmd) {
  // spi_txd(HSPI, 9, (0x00 << 8) | cmd);
  spi_lcd_9bit_write(ESPHSPI, CMD, cmd);
  // spi_mast_byte_write(HSPI, 0xFF);
}

void HX8357::write_command_data(uint8_t cmd, uint8_t *data, uint8_t lenInBytes) {
  write_command(cmd);

  // write_data(data, lenInBytes);
  for (int i = 0; i < lenInBytes; i++) {
    write_data(data[i]);
  }
}

void HX8357::write_data(uint8_t data) {
  spi_lcd_9bit_write(ESPHSPI, DATA, data);
}

void HX8357::write_data(uint8_t *data, uint8_t lenInBytes) {
  // spi_txd(HSPI, 9, (0x01 << 8) | data);
  // spi_lcd_9bit_write(HSPI, DATA, data);

	// uint32 regvalue;
	// uint8 bytetemp;
  uint8 idx = 0;

  // SET_PERI_REG_MASK(SPI_USER(HSPI), SPI_CS_HOLD);
  // SET_PERI_REG_BITS(SPI_USER1(HSPI), SPI_USR_MOSI_BITLEN, 15, SPI_USR_MOSI_BITLEN_S);

  // WRITE_PERI_REG(SPI_USER1(HSPI), ((0&SPI_USR_MISO_BITLEN)<<SPI_USR_MISO_BITLEN_S));
  // WRITE_PERI_REG(SPI_USER1(HSPI), ((0&SPI_USR_ADDR_BITLEN)<<SPI_USR_ADDR_BITLEN_S));

  do {
    spi_lcd_9bit_write(ESPHSPI, DATA, data[idx]);
  } while (++idx < lenInBytes);
  // // Enable MOSI
  // SET_PERI_REG_MASK(SPI_USER(HSPI), SPI_USR_MOSI);
  //
  // // Load send buffer
  // do {
  //     bytetemp=(data[idx]>>1)|0x80;
  //
  //     regvalue= ((8&SPI_USR_COMMAND_BITLEN)<<SPI_USR_COMMAND_BITLEN_S)|((uint32)bytetemp);		//configure transmission variable,9bit transmission length and first 8 command bit
  //     regvalue|=BIT15;        //write the 9th bit
  //
  //     WRITE_PERI_REG((SPI_W0(HSPI) + (idx << 2)), regvalue);
  //     // WRITE_PERI_REG(SPI_USER2(HSPI), regvalue);
  // } while (++idx < lenInBytes);
  // // Set data send buffer length.Max data length 64 bytes.
  // SET_PERI_REG_BITS(SPI_USER1(HSPI), SPI_USR_MOSI_BITLEN, lenInBytes - 1, SPI_USR_MOSI_BITLEN_S);
  //
  // // Start send data
  // SET_PERI_REG_MASK(SPI_CMD(HSPI), SPI_USR);
  // // Wait for transmit done
  // while (!(READ_PERI_REG(SPI_SLAVE(HSPI))&SPI_TRANS_DONE));
  // CLEAR_PERI_REG_MASK(SPI_SLAVE(HSPI), SPI_TRANS_DONE);
}

void HX8357::drawPixel(int16_t x, int16_t y, uint16_t color) {
   setAddrWindow(x, y, x, y);
 	 write_data_rgb(color, 1);
 }


// BITMAP / XBITMAP / GRAYSCALE / RGB BITMAP FUNCTIONS ---------------------

/**************************************************************************/
/*!
   @brief      Draw a PROGMEM-resident 1-bit image at the specified (x,y)
   position, using the specified foreground color (unset bits are transparent).
*/
/**************************************************************************/
void HX8357::drawBitmap(int16_t x, int16_t y, const uint8_t bitmap[],
                              int16_t w, int16_t h, uint16_t color) {

  int16_t byteWidth = (w + 7) / 8; // Bitmap scanline pad = whole byte
  uint8_t byte = 0;

  startWrite();
  for (int16_t j = 0; j < h; j++, y++) {
    for (int16_t i = 0; i < w; i++) {
      if (i & 7)
        byte <<= 1;
      else
        byte = pgm_read_byte(&bitmap[j * byteWidth + i / 8]);
      if (byte & 0x80)
        writePixel(x + i, y, color);
    }
  }
  endWrite();
}

/**************************************************************************/
/*!
   @brief      Draw a PROGMEM-resident 1-bit image at the specified (x,y)
   position, using the specified foreground (for set bits) and background (unset
   bits) colors.
*/
/**************************************************************************/
void HX8357::drawBitmap(int16_t x, int16_t y, const uint8_t bitmap[],
                              int16_t w, int16_t h, uint16_t color,
                              uint16_t bg) {

  int16_t byteWidth = (w + 7) / 8; // Bitmap scanline pad = whole byte
  uint8_t byte = 0;

  startWrite();
  for (int16_t j = 0; j < h; j++, y++) {
    for (int16_t i = 0; i < w; i++) {
      if (i & 7)
        byte <<= 1;
      else
        byte = pgm_read_byte(&bitmap[j * byteWidth + i / 8]);
      writePixel(x + i, y, (byte & 0x80) ? color : bg);
    }
  }
  endWrite();
}

/**************************************************************************/
/*!
   @brief      Draw a RAM-resident 1-bit image at the specified (x,y) position,
   using the specified foreground color (unset bits are transparent).
*/
/**************************************************************************/
void HX8357::drawBitmap(int16_t x, int16_t y, uint8_t *bitmap, int16_t w,
                              int16_t h, uint16_t color) {

  int16_t byteWidth = (w + 7) / 8; // Bitmap scanline pad = whole byte
  uint8_t byte = 0;

  startWrite();
  for (int16_t j = 0; j < h; j++, y++) {
    for (int16_t i = 0; i < w; i++) {
      if (i & 7)
        byte <<= 1;
      else
        byte = bitmap[j * byteWidth + i / 8];
      if (byte & 0x80)
        writePixel(x + i, y, color);
    }
  }
  endWrite();
}

/**************************************************************************/
/*!
   @brief      Draw a RAM-resident 1-bit image at the specified (x,y) position,
   using the specified foreground (for set bits) and background (unset bits)
   colors.
*/
/**************************************************************************/
void HX8357::drawBitmap(int16_t x, int16_t y, uint8_t *bitmap, int16_t w,
                              int16_t h, uint16_t color, uint16_t bg) {

  int16_t byteWidth = (w + 7) / 8; // Bitmap scanline pad = whole byte
  uint8_t byte = 0;

  startWrite();
  for (int16_t j = 0; j < h; j++, y++) {
    for (int16_t i = 0; i < w; i++) {
      if (i & 7)
        byte <<= 1;
      else
        byte = bitmap[j * byteWidth + i / 8];
      writePixel(x + i, y, (byte & 0x80) ? color : bg);
    }
  }
  endWrite();
}

/**************************************************************************/
/*!
   @brief   Draw a PROGMEM-resident 8-bit image (grayscale) at the specified
   (x,y) pos. Specifically for 8-bit display devices such as IS31FL3731; no
   color reduction/expansion is performed.
*/
/**************************************************************************/
void HX8357::drawGrayscaleBitmap(int16_t x, int16_t y,
                                       const uint8_t bitmap[], int16_t w,
                                       int16_t h) {
  startWrite();
  for (int16_t j = 0; j < h; j++, y++) {
    for (int16_t i = 0; i < w; i++) {
      writePixel(x + i, y, (uint8_t)pgm_read_byte(&bitmap[j * w + i]));
    }
  }
  endWrite();
}

/**************************************************************************/
/*!
   @brief   Draw a RAM-resident 8-bit image (grayscale) at the specified (x,y)
   pos. Specifically for 8-bit display devices such as IS31FL3731; no color
   reduction/expansion is performed.
*/
/**************************************************************************/
void HX8357::drawGrayscaleBitmap(int16_t x, int16_t y, uint8_t *bitmap,
                                       int16_t w, int16_t h) {
  startWrite();
  for (int16_t j = 0; j < h; j++, y++) {
    for (int16_t i = 0; i < w; i++) {
      writePixel(x + i, y, bitmap[j * w + i]);
    }
  }
  endWrite();
}

/**************************************************************************/
/*!
   @brief   Draw a PROGMEM-resident 8-bit image (grayscale) with a 1-bit mask
   (set bits = opaque, unset bits = clear) at the specified (x,y) position.
   BOTH buffers (grayscale and mask) must be PROGMEM-resident.
*/
/**************************************************************************/
void HX8357::drawGrayscaleBitmap(int16_t x, int16_t y,
                                       const uint8_t bitmap[],
                                       const uint8_t mask[], int16_t w,
                                       int16_t h) {
  int16_t bw = (w + 7) / 8; // Bitmask scanline pad = whole byte
  uint8_t byte = 0;
  startWrite();
  for (int16_t j = 0; j < h; j++, y++) {
    for (int16_t i = 0; i < w; i++) {
      if (i & 7)
        byte <<= 1;
      else
        byte = pgm_read_byte(&mask[j * bw + i / 8]);
      if (byte & 0x80) {
        writePixel(x + i, y, (uint8_t)pgm_read_byte(&bitmap[j * w + i]));
      }
    }
  }
  endWrite();
}

/**************************************************************************/
/*!
   @brief   Draw a RAM-resident 8-bit image (grayscale) with a 1-bit mask
   (set bits = opaque, unset bits = clear) at the specified (x,y) position.
   BOTH buffers (grayscale and mask) must be RAM-residentt, no mix-and-match
   Specifically for 8-bit display devices such as IS31FL3731; no color
   reduction/expansion is performed.
*/
/**************************************************************************/
void HX8357::drawGrayscaleBitmap(int16_t x, int16_t y, uint8_t *bitmap,
                                       uint8_t *mask, int16_t w, int16_t h) {
  int16_t bw = (w + 7) / 8; // Bitmask scanline pad = whole byte
  uint8_t byte = 0;
  startWrite();
  for (int16_t j = 0; j < h; j++, y++) {
    for (int16_t i = 0; i < w; i++) {
      if (i & 7)
        byte <<= 1;
      else
        byte = mask[j * bw + i / 8];
      if (byte & 0x80) {
        writePixel(x + i, y, bitmap[j * w + i]);
      }
    }
  }
  endWrite();
}

/**************************************************************************/
/*!
   @brief   Draw a PROGMEM-resident 16-bit image (RGB 5/6/5) at the specified
   (x,y) position. For 16-bit display devices; no color reduction performed.
*/
/**************************************************************************/
void HX8357::drawRGBBitmap(int16_t x, int16_t y, const uint16_t bitmap[],
                                 int16_t w, int16_t h) {
  startWrite();
  for (int16_t j = 0; j < h; j++, y++) {
    for (int16_t i = 0; i < w; i++) {
      writePixel(x + i, y, pgm_read_word(&bitmap[j * w + i]));
    }
  }
  endWrite();
}

/**************************************************************************/
/*!
   @brief   Draw a RAM-resident 16-bit image (RGB 5/6/5) at the specified (x,y)
   position. For 16-bit display devices; no color reduction performed.
*/
/**************************************************************************/
void HX8357::drawRGBBitmap(int16_t x, int16_t y, uint16_t *bitmap,
                                 int16_t w, int16_t h) {
  startWrite();
  for (int16_t j = 0; j < h; j++, y++) {
    for (int16_t i = 0; i < w; i++) {
      writePixel(x + i, y, bitmap[j * w + i]);
    }
  }
  endWrite();
}

/**************************************************************************/
/*!
   @brief   Draw a PROGMEM-resident 16-bit image (RGB 5/6/5) with a 1-bit mask
   (set bits = opaque, unset bits = clear) at the specified (x,y) position. BOTH
   buffers (color and mask) must be PROGMEM-resident. For 16-bit display
   devices; no color reduction performed.
*/
/**************************************************************************/
void HX8357::drawRGBBitmap(int16_t x, int16_t y, const uint16_t bitmap[],
                                 const uint8_t mask[], int16_t w, int16_t h) {
  int16_t bw = (w + 7) / 8; // Bitmask scanline pad = whole byte
  uint8_t byte = 0;
  startWrite();
  for (int16_t j = 0; j < h; j++, y++) {
    for (int16_t i = 0; i < w; i++) {
      if (i & 7)
        byte <<= 1;
      else
        byte = pgm_read_byte(&mask[j * bw + i / 8]);
      if (byte & 0x80) {
        writePixel(x + i, y, pgm_read_word(&bitmap[j * w + i]));
      }
    }
  }
  endWrite();
}

/**************************************************************************/
/*!
   @brief   Draw a RAM-resident 16-bit image (RGB 5/6/5) with a 1-bit mask (set
   bits = opaque, unset bits = clear) at the specified (x,y) position. BOTH
   buffers (color and mask) must be RAM-resident. For 16-bit display devices; no
   color reduction performed.
*/
/**************************************************************************/
void HX8357::drawRGBBitmap(int16_t x, int16_t y, uint16_t *bitmap,
                                 uint8_t *mask, int16_t w, int16_t h) {
  int16_t bw = (w + 7) / 8; // Bitmask scanline pad = whole byte
  uint8_t byte = 0;
  startWrite();
  for (int16_t j = 0; j < h; j++, y++) {
    for (int16_t i = 0; i < w; i++) {
      if (i & 7)
        byte <<= 1;
      else
        byte = mask[j * bw + i / 8];
      if (byte & 0x80) {
        writePixel(x + i, y, bitmap[j * w + i]);
      }
    }
  }
  endWrite();
}

void HX8357::setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
  // uint8_t data[4] = {0};

  // data[0] = x0 >> 8;
  // data[1] = x0 & 0xFF;
  // data[2] = x1 >> 8;
  // data[3] = x1 & 0xFF;
  // write_command_data(ILI9488_CASET, data, 4);

  write_command(HX8357_CASET); // Column addr set
  write_data(x0 >> 8);
  write_data(x0 & 0xFF);     // XSTART
  write_data(x1 >> 8);
  write_data(x1 & 0xFF);     // XEND

  write_command(HX8357_PASET); // Row addr set
  write_data(y0>>8);
  write_data(y0);     // YSTART
  write_data(y1>>8);
  write_data(y1);     // YEND

  // data[0] = y0>>8;
  // data[1] = y0;
  // data[2] = y1>>8;
  // data[3] = y1;
  // write_command_data(ILI9488_PASET, data, 4);

  write_command(HX83h7_RAMWR); // write to RAM
}

void HX8357::write_data_rgb(uint16_t color, uint32_t repeats){

  #ifdef USE_3_BIT_COLORS
  while(repeats > 0)
  {
    write_data((uint8_t)color);
    repeats--;
  }
  #endif
  #ifdef USE_16_BIT_COLORS
  // uint16_t r = (color & 0x00FF0000)>>16;
  // r&=0x1FC;
  // uint16_t g = (color & 0x0000FF00)>>8;
  // g&=0x1FC;
  // uint16_t b = (color & 0x000000FF);
  // b&=0x1FC;

  uint8_t data[3] = {0};

  int r = ((color >> 11) & 0x1F);  // Extract the 5 R bits
  int g = ((color >> 5) & 0x3F);   // Extract the 6 G bits
  int b = ((color) & 0x1F);        // Extract the 5 B bits

  while(repeats > 0)
  {

    // FF FF
    // 0000 0000 0000 0000

    //write_data(r);
    //write_data(g);
    //write_data(b);
    //map(0x00, 0xFC)
    //write_data(0x0);
    //write_data(0x0);
    //write_data(0x0);
    data[0] = r;
    data[1] = g;
    data[2] = b;
    write_data(data, 3);
    repeats--;
  }
  #endif

 };

 void HX8357::setRotation(uint8_t m) {

	uint8_t data;
	uint8_t rotation = m % 4; // can't be higher than 3
	switch (rotation) {
	case 0:
		data = MADCTL_MX | MADCTL_BGR;
		_width  = TFTWIDTH;
		_height = TFTHEIGHT;
		break;
	case 1:
		data = MADCTL_MV | MADCTL_BGR;
		_width  = TFTHEIGHT;
		_height = TFTWIDTH;
		break;
	case 2:
		data = MADCTL_MY | MADCTL_BGR;
		_width  = TFTWIDTH;
		_height = TFTHEIGHT;
		break;
	case 3:
		data = MADCTL_MX | MADCTL_MY | MADCTL_MV | MADCTL_BGR;
		_width  = TFTHEIGHT;
		_height = TFTWIDTH;
		break;
	}
  write_command(HX8357_MADCTL);
  write_data(data);
}
