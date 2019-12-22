/**
* Mirrored_LEDBackpack.cpp - Mirroring extension for Adafruit LED Bacpack library
*/

#include <Arduino.h>
#include <Wire.h>
#include <ht16k33.h>
#include <Adafruit_LEDBackpack.h>
#include <Adafruit_GFX.h>
#include <Mirrored_LEDBackpack.h>

Adafruit_X_Mirrored_8x8matrix::Adafruit_X_Mirrored_8x8matrix(void) : Adafruit_GFX(8, 8) {
}

void Adafruit_X_Mirrored_8x8matrix::drawPixel(int16_t x, int16_t y, uint16_t color) {
  if ((y < 0) || (y >= 8)) return;
  if ((x < 0) || (x >= 8)) return;

 // check rotation, move pixel around if necessary
  switch (getRotation()) {
  case 1:
    _swap_int16_t(x, y);
    x = 8 - x - 1;
    break;
  case 2:
    x = 8 - x - 1;
    y = 8 - y - 1;
    break;
  case 3:
    _swap_int16_t(x, y);
    y = 8 - y - 1;
    break;
  }

  x = 8 - x - 1; // X pixel mirrored

    if (color) {
    displaybuffer[y] |= 1 << x;
  } else {
    displaybuffer[y] &= ~(1 << x);
  }
}


Adafruit_X_Mirrored_8x16minimatrix::Adafruit_X_Mirrored_8x16minimatrix(void) : Adafruit_GFX(8, 16) {
}

void Adafruit_X_Mirrored_8x16minimatrix::drawPixel(int16_t x, int16_t y, uint16_t color) {

  if ((y < 0) || (x < 0)) return;
  if ((getRotation() % 2 == 0) && ((y >= 16) || (x >= 8))) return;
  if ((getRotation() % 2 == 1) && ((x >= 16) || (y >= 8))) return;


 // check rotation, move pixel around if necessary
  switch (getRotation()) {
  case 2:
    if (y >= 8) {
      x += 8;
      y -= 8; 
    }
     _swap_int16_t(x, y);
    break;
  case 3:
    x = 16 - x - 1;
    if (x >= 8) {
      x -= 8;
      y += 8; 
    }
    break;
  case 0:
    y = 16 - y - 1;
    x = 8 - x - 1;
    if (y >= 8) {
      x += 8;
      y -= 8; 
    }
     _swap_int16_t(x, y);
    break;
  case 1:
    //y = 8 - y - 1;
    if (x >= 8) {
      x -= 8;
      y += 8; 
    }
    break;
  }

    //y = 16 - y - 1; // X pixel mirrored

  if (color) {
    displaybuffer[x] |= 1 << y;
  } else {
    displaybuffer[x] &= ~(1 << y);
  }
}
