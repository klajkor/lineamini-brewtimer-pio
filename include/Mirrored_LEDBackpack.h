/**
* Mirrored_LEDBackpack.h - Mirroring extension for Adafruit LED Bacpack library
*/

#ifndef _MIRRORED_LED_BACKPACK_H
#define _MIRRORED_LED_BACKPACK_H

#include <Arduino.h>
#include <Wire.h>
#include <ht16k33.h>
#include <Adafruit_LEDBackpack.h>
#include <Adafruit_GFX.h>


#ifndef _swap_int16_t
#define _swap_int16_t(a, b) { int16_t t = a; a = b; b = t; }
#endif

// mirror classes - this is required becuase of different wiring of LED matrix

class Adafruit_X_Mirrored_8x8matrix: public Adafruit_LEDBackpack, public Adafruit_GFX {
  
  public:
  Adafruit_X_Mirrored_8x8matrix(void);

  void drawPixel(int16_t x, int16_t y, uint16_t color);

  private:

};

class Adafruit_X_Mirrored_8x16minimatrix : public Adafruit_LEDBackpack, public Adafruit_GFX {
 public:
  Adafruit_X_Mirrored_8x16minimatrix(void);

  void drawPixel(int16_t x, int16_t y, uint16_t color);

 private:
};

#endif  //_MIRRORED_LED_BACKPACK_H