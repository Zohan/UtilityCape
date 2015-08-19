#include <Adafruit_NeoPixel.h>

// Parameter 1 = number of pixels in strip
// Parameter 2 = pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_RGB     Pixels are wired for RGB bitstream
//   NEO_GRB     Pixels are wired for GRB bitstream
//   NEO_KHZ400  400 KHz bitstream (e.g. FLORA pixels)
//   NEO_KHZ800  800 KHz bitstream (e.g. High Density LED strip)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(60, 6, NEO_GRB + NEO_KHZ800);
#define ALTERNATE_COLUMNS 1
#define SIZEX 6
#define SIZEY 10

void setup() {
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
}

void loop() {
  // Some example procedures showing how to display to the pixels:
  colorWipe(strip.Color(255, 0, 0), 50); // Red
  colorWipe(strip.Color(0, 255, 0), 50); // Green
  //colorWipe(strip.Color(0, 0, 255), 50); // Blue
  rainbow(20);
  //rainbowCycle(20);
}

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<SIZEX; i++) {
    for(uint16_t j=0; j<SIZEY; j++) {
      setGridPixelColor(i, j, c);
      strip.show();
      delay(wait);
    }
  }
}

void rainbow(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256; j++) {
    for(i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(getPixelCoords(i), Wheel((i+j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< strip.numPixels(); i++) {
      strip.setPixelColor(getPixelCoords(i), Wheel(((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

uint16_t getPixelNumber(int x, int y) {
  uint16_t pixel;
  if(ALTERNATE_COLUMNS && x%2 == 1)
  {
    pixel = (x*10) + (9-y);
  } else {
    pixel = (x*10) + y;
  }
  return pixel;
}

uint16_t getPixelCoords(int i) {
  uint16_t pixel;
  int x = i/10;
  int y = i%10;
  if(ALTERNATE_COLUMNS && i%2 == 1) {
    pixel = (x*10) + (9-y);
  } else {
    pixel = (x*10) + y;
  }
  return pixel;
}

void setGridPixelColor(int x, int y, uint16_t c) {
  uint16_t pixel;
  if(ALTERNATE_COLUMNS && x%2 == 1) {
    pixel = (x*10) + (9-y);
  } else {
    pixel = (x*10) + y;
  }
 
  strip.setPixelColor(pixel, c);
  strip.show();
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  if(WheelPos < 85) {
   return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if(WheelPos < 170) {
   WheelPos -= 85;
   return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
   WheelPos -= 170;
   return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}

