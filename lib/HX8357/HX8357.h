#ifndef _HX8357_
#define _HX8357_

#if ARDUINO >= 100
 #include "Arduino.h"
 #include "Print.h"
#else
 #include "WProgram.h"
#endif
#include <Adafruit_GFX.h>

// #define USE_3_BIT_COLORS
#define USE_16_BIT_COLORS

#define TFTWIDTH  320
#define TFTHEIGHT  480

#define LANDSCAPE_LEFT 3 // flat flex connector is considered BOTTOM
#define LANDSCAPE_RIGHT 1 // flat flex connector is considered TOP


class HX8357 : public Adafruit_GFX {

private:
void write_command(uint8_t cmd);
void write_command_data(uint8_t cmd, uint8_t *data, uint8_t lenInBytes);
void write_data(uint8_t data);
void write_data(uint8_t *data, uint8_t lenInBytes);
void write_data_rgb(uint16_t color, uint32_t repeats);
void setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);

public:

  HX8357();

  void begin();

  void drawPixel(int16_t x, int16_t y, uint16_t color);
  
  void setRotation(uint8_t m);
  
  void drawBitmap(int16_t x, int16_t y, const uint8_t bitmap[], int16_t w,
                  int16_t h, uint16_t color);
  void drawBitmap(int16_t x, int16_t y, const uint8_t bitmap[], int16_t w,
                  int16_t h, uint16_t color, uint16_t bg);
  void drawBitmap(int16_t x, int16_t y, uint8_t *bitmap, int16_t w, int16_t h,
                  uint16_t color);
  void drawBitmap(int16_t x, int16_t y, uint8_t *bitmap, int16_t w, int16_t h,
                  uint16_t color, uint16_t bg);
  void drawXBitmap(int16_t x, int16_t y, const uint8_t bitmap[], int16_t w,
                   int16_t h, uint16_t color);
  void drawGrayscaleBitmap(int16_t x, int16_t y, const uint8_t bitmap[],
                           int16_t w, int16_t h);
  void drawGrayscaleBitmap(int16_t x, int16_t y, uint8_t *bitmap, int16_t w,
                           int16_t h);
  void drawGrayscaleBitmap(int16_t x, int16_t y, const uint8_t bitmap[],
                           const uint8_t mask[], int16_t w, int16_t h);
  void drawGrayscaleBitmap(int16_t x, int16_t y, uint8_t *bitmap, uint8_t *mask,
                           int16_t w, int16_t h);
  void drawRGBBitmap(int16_t x, int16_t y, const uint16_t bitmap[], int16_t w,
                     int16_t h);
  void drawRGBBitmap(int16_t x, int16_t y, uint16_t *bitmap, int16_t w,
                     int16_t h);
  void drawRGBBitmap(int16_t x, int16_t y, const uint16_t bitmap[],
                     const uint8_t mask[], int16_t w, int16_t h);
  void drawRGBBitmap(int16_t x, int16_t y, uint16_t *bitmap, uint8_t *mask,
                     int16_t w, int16_t h);
};
#endif
