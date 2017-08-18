/*-------------------------------------------------------------------------
  Arduino library to control single and tiled matrices of WS2811- and
  WS2812-based RGB LED devices such as the Adafruit NeoPixel Shield or
  displays assembled from NeoPixel strips, making them compatible with
  the Adafruit_GFX graphics library.  Requires both the Adafruit_NeoPixel
  and Adafruit_GFX libraries.

  Written by Phil Burgess / Paint Your Dragon for Adafruit Industries.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing products
  from Adafruit!

  -------------------------------------------------------------------------
  This file is part of the Adafruit NeoMatrix library.

  NeoMatrix is free software: you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as
  published by the Free Software Foundation, either version 3 of
  the License, or (at your option) any later version.

  NeoMatrix is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with NeoMatrix.  If not, see
  <http://www.gnu.org/licenses/>.
  -------------------------------------------------------------------------*/

#include "Adafruit_NeoPixel.h"
#include "Adafruit_NeoMatrix.h"
#include "gamma.h"
#ifdef __AVR__
 #include <avr/pgmspace.h>
#elif defined(ESP8266)
 #include <pgmspace.h>
#else
 #ifndef pgm_read_byte
  #define pgm_read_byte(addr) (*(const unsigned char *)(addr))
 #endif
#endif

#define COLLAR_OFFSET 10 // For the 10 LEDs on the collar

// Constructor for single matrix:
Adafruit_NeoMatrix::Adafruit_NeoMatrix(int w, int h, uint8_t pin,
  uint8_t matrixType, uint8_t ledType) : Adafruit_GFX(w, h),
  Adafruit_NeoPixel(w * h + COLLAR_OFFSET, pin, ledType), type(matrixType), matrixWidth(w),
  matrixHeight(h), tilesX(0), tilesY(0), remapFn(NULL) { }

// Constructor for tiled matrices:
Adafruit_NeoMatrix::Adafruit_NeoMatrix(uint8_t mW, uint8_t mH, uint8_t tX,
  uint8_t tY, uint8_t pin, uint8_t matrixType, uint8_t ledType) :
  Adafruit_GFX(mW * tX, mH * tY), Adafruit_NeoPixel(mW * mH * tX * tY, pin,
  ledType), type(matrixType), matrixWidth(mW), matrixHeight(mH), tilesX(tX),
  tilesY(tY), remapFn(NULL) { }

// Expand 16-bit input color (Adafruit_GFX colorspace) to 24-bit (NeoPixel)
// (w/gamma adjustment)
static uint32_t expandColor(uint16_t color) {
  return ((uint32_t)pgm_read_byte(&gamma5[ color >> 11       ]) << 16) |
         ((uint32_t)pgm_read_byte(&gamma6[(color >> 5) & 0x3F]) <<  8) |
                    pgm_read_byte(&gamma5[ color       & 0x1F]);
}

// Downgrade 24-bit color to 16-bit (add reverse gamma lookup here?)
uint16_t Adafruit_NeoMatrix::Color(uint8_t r, uint8_t g, uint8_t b) {
  return ((uint16_t)(r & 0xF8) << 8) |
         ((uint16_t)(g & 0xFC) << 3) |
                    (b         >> 3);
}

void Adafruit_NeoMatrix::drawCollarPixel(int16_t x, uint16_t color) {
  setPixelColor(x, expandColor(color));
}

void Adafruit_NeoMatrix::drawPixel(int16_t x, int16_t y, uint16_t color) {

  if((x < 0) || (y < 0) || (x >= _width) || (y >= _height)) return;

  int16_t t;
  switch(rotation) {
   case 1:
    t = x;
    x = WIDTH  - 1 - y;
    y = t;
    break;
   case 2:
    x = WIDTH  - 1 - x;
    y = HEIGHT - 1 - y;
    break;
   case 3:
    t = x;
    x = y;
    y = HEIGHT - 1 - t;
    break;
  }

  int tileOffset = 0, pixelOffset;

  if(remapFn) { // Custom X/Y remapping function
    pixelOffset = (*remapFn)(x, y);
  } else {      // Standard single matrix or tiled matrices

    uint8_t  corner = type & NEO_MATRIX_CORNER;
    uint16_t minor, major, majorScale;

    if(tilesX) { // Tiled display, multiple matrices
      uint16_t tile;

      minor = x / matrixWidth;            // Tile # X/Y; presume row major to
      major = y / matrixHeight,           // start (will swap later if needed)
      x     = x - (minor * matrixWidth);  // Pixel X/Y within tile
      y     = y - (major * matrixHeight); // (-* is less math than modulo)

      // Determine corner of entry, flip axes if needed
      if(type & NEO_TILE_RIGHT)  minor = tilesX - 1 - minor;
      if(type & NEO_TILE_BOTTOM) major = tilesY - 1 - major;

      // Determine actual major axis of tiling
      if((type & NEO_TILE_AXIS) == NEO_TILE_ROWS) {
        majorScale = tilesX;
      } else {
        swap(major, minor);
        majorScale = tilesY;
      }

      // Determine tile number
      if((type & NEO_TILE_SEQUENCE) == NEO_TILE_PROGRESSIVE) {
        // All tiles in same order
        tile = major * majorScale + minor;
      } else {
        // Zigzag; alternate rows change direction.  On these rows,
        // this also flips the starting corner of the matrix for the
        // pixel math later.
        if(major & 1) {
          corner ^= NEO_MATRIX_CORNER;
          tile = (major + 1) * majorScale - 1 - minor;
        } else {
          tile =  major      * majorScale     + minor;
        }
      }

      // Index of first pixel in tile
      tileOffset = tile * matrixWidth * matrixHeight;

    } // else no tiling (handle as single tile)

    // Find pixel number within tile
    minor = x; // Presume row major to start (will swap later if needed)
    major = y;

    // Determine corner of entry, flip axes if needed
    if(corner & NEO_MATRIX_RIGHT)  minor = matrixWidth  - 1 - minor;
    if(corner & NEO_MATRIX_BOTTOM) major = matrixHeight - 1 - major;

    // Determine actual major axis of matrix
    if((type & NEO_MATRIX_AXIS) == NEO_MATRIX_ROWS) {
      majorScale = matrixWidth;
    } else {
      swap(major, minor);
      majorScale = matrixHeight;
    }

    // Determine pixel number within tile/matrix
    if((type & NEO_MATRIX_SEQUENCE) == NEO_MATRIX_PROGRESSIVE) {
      // All lines in same order
      pixelOffset = major * majorScale + minor;
    } else {
      // Zigzag; alternate rows change direction.
      if(major & 1) pixelOffset = (major + 1) * majorScale - 1 - minor;
      else          pixelOffset =  major      * majorScale     + minor;
    }
  }

  setPixelColor(tileOffset + pixelOffset + COLLAR_OFFSET, expandColor(color));
}

void Adafruit_NeoMatrix::fillScreen(uint16_t color) {
  uint16_t i, n;
  uint32_t c24;

  c24 = expandColor(color);
  n   = numPixels();
  for(i=0; i<n; i++) setPixelColor(i + COLLAR_OFFSET, c24);
}

void Adafruit_NeoMatrix::setRemapFunction(uint16_t (*fn)(uint16_t, uint16_t)) {
  remapFn = fn;
}
