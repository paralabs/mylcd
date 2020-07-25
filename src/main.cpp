#define PGM_READ_UNALIGNED 0

#include <Arduino.h>

#include "../lib/HX8357/HX8357.h"

HX8357 display = HX8357();

uint8_t orientation = 0;
uint16_t r       = 0x01;
uint16_t g       = 0x01;
uint16_t b       = 0x01;
int32_t color   = 111;
uint8_t depth    = 0x0001;
uint8_t t_size   = 0x0A;

void setup()
{
  Serial.begin(1000000);
  while (!Serial) {};
  Serial.println("Initializing Display...");
  display.begin();
  display.setRotation(orientation); // 0 -3
  Serial.println("Hacked M200 Display.");
}

void loop()
{

  while (true)
  {
    display.setRotation(orientation); // 0 -3
    color += t_size + (r + g + b) * depth;
    display.drawCircle(t_size, t_size, depth, color);

    r += (t_size + depth) + 1;
    g += (t_size + depth) + 1;
    b += (t_size + depth) + 1;
    orientation = orientation + 1;
    depth = depth + 10;
    t_size = t_size + 5;
    
    display.setRotation(orientation); // 0 -3
    color += t_size + (r + g + b) * depth;
    display.drawCircle(t_size, t_size, depth, color);
  }
}
