/*
 Cape of Life
 Adapted from http://pastebin.com/f22bfe94d and Adafruit Neopixel tutorial.
 In this case; 0 is present, 1 is future, 2+ is past.
*/
#include <Adafruit_CircuitPlayground.h>
#include <Adafruit_GFX.h>
#include <Adafruit_NeoMatrix.h>
#include <avr/interrupt.h>
#include "Adafruit_ZeroTimer.h"
#include <FastLED.h>

// Columns were alternated between running up the cape and down the cape for wiring purposes. With this on, every other row will have the output flipped.
#define DELAY 0
#define SIZEX 10
#define SIZEY 42
#define COLLAR_OFFSET 20
#define MATRIX_DIMENSIONS SIZEX*SIZEY
// This determines how 'smooth' transitions between worlds will be
#define COLORCYCLEAMOUNT 4

#define LED_PIN A1
const int freq = 100;
const int ledButton = 4;    // the number of the pushbutton pin
const int heatingButton = 19;    // the number of the pushbutton pin
const int heaterPin = SCL;    // the number of the pushbutton pin
const int heaterLedPin = 13;
int heaterLedState = LOW;         // the current state of the output pin
int ledButtonState;             // the current reading from the input pin
int heaterButtonState;             // the current reading from the input pin
int lastLedButtonState = 0;   // the previous reading from the input pin
int lastHeaterButtonState = 0;   // the previous reading from the input pin
long lastDebounceTime = 0;  // the last time the output pin was toggled
long debounceDelay = 32;    // the debounce time; increase if the output flickers

int ledMode = 1;
byte heaterMode = 0;
byte collarLed = 20;
byte collarColor = 0;

byte world[SIZEX][SIZEY][3], oldColor, rgbOld[3], rgbNew[3];
boolean firstCycle = true;
boolean ledModeInterrupted = false;
long density = 22;

// Fire palette
const TProgmemRGBPalette16 InverseHeatColors_p FL_PROGMEM =
{
    0x000000,
    0x000033, 0x000066, 0x000099, 0x0000CC, 0x0000FF,
    0x0033FF, 0x0066FF, 0x0099FF, 0x00CCFF, 0x00FFFF,
    0x33FFFF, 0x66FFFF, 0x99FFFF, 0xCCFFFF, 0xFFFFFF
};
CRGBPalette16 gPal;
bool gReverseDirection = true;
 
// Parameter 1 = number of pixels in strip
// Parameter 2 = pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_RGB     Pixels are wired for RGB bitstream
//   NEO_GRB     Pixels are wired for GRB bitstream
//   NEO_KHZ400  400 KHz bitstream (e.g. FLORA pixels)
//   NEO_KHZ800  800 KHz bitstream (e.g. High Density LED strip)
// We claim the strip is 10 larger than it is.
Adafruit_NeoMatrix matrix = Adafruit_NeoMatrix(SIZEX, SIZEY+COLLAR_OFFSET/SIZEX, LED_PIN,
  NEO_MATRIX_TOP     + NEO_MATRIX_RIGHT +
  NEO_MATRIX_ROWS + NEO_MATRIX_ZIGZAG,
  NEO_GRB            + NEO_KHZ800);


void collarCounter(){
  noInterrupts();
  int collarX = collarLed/2;
  int collarY = collarLed%2;
//    Serial.print("x:");
//    Serial.print(collarX);
//    Serial.print(" y:");
//    Serial.println(collarY);
  matrix.drawPixel(collarX, 42+collarY, Wheel(collarColor));
  collarLed++;
  if(collarLed >= COLLAR_OFFSET) {
    collarLed = 0;
    collarColor = random(255);
  }
  matrix.show();
  interrupts();
}

// timer
Adafruit_ZeroTimer zerotimer = Adafruit_ZeroTimer(4);
void TC4_Handler() {
  Adafruit_ZeroTimer::timerHandler(4);
}

volatile int freq_count = 0;
void TimerCallback0(void)
{
  int reading = CircuitPlayground.leftButton();
  int reading2 = CircuitPlayground.rightButton();
  if (reading != lastLedButtonState) {
//    Serial.println(reading);
    lastDebounceTime = millis();
  }
  if (reading2 != lastHeaterButtonState) {
    lastDebounceTime = millis();
  }
  if ((millis() - lastDebounceTime) > debounceDelay) {
//    Serial.println("Debounce!");
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:
    // if the button state has changed:
    if (reading != ledButtonState) {
      ledButtonState = reading;

      // only toggle the LED if the new button state is HIGH
      if (ledButtonState) {
        ledMode++;
        Serial.println("Changing LED mode!");
        ledModeInterrupted = true;
      }
    }
    if (reading2 != heaterButtonState) {
      heaterButtonState = reading2;
      
      if (heaterButtonState) {
        heaterMode++;
        switch(heaterMode) {
          case 0:
          Serial.println("Heater off!");
          analogWrite(heaterPin, 0);
          digitalWrite(heaterLedPin, LOW);
          break;
          
          case 1:
          Serial.println("Heater hi!");
          analogWrite(heaterPin, 255);
          digitalWrite(heaterLedPin, HIGH);
          break;
          
          default:
          heaterMode = 0;
          analogWrite(heaterPin, 0);
          digitalWrite(heaterLedPin, LOW);
          Serial.println("Heater off!");
          
        }
      }
    }
  }
  lastLedButtonState = reading;
  lastHeaterButtonState = reading2;

  if (freq_count <= freq-22){
    freq_count += 1;
  } else {
    freq_count = 0;
    collarCounter();
    Serial.println("Tick");
  }
  
  
}

void setupTimer() {
  uint16_t divider  = 1;
  uint16_t compare = 0;
  tc_clock_prescaler prescaler = TC_CLOCK_PRESCALER_DIV1;
  if ((freq < 24000000) && (freq > 800)) {
    divider = 1;
    prescaler = TC_CLOCK_PRESCALER_DIV1;
    compare = 48000000/freq;
  } else if (freq > 400) {
    divider = 2;
    prescaler = TC_CLOCK_PRESCALER_DIV2;
    compare = (48000000/2)/freq;
  } else if (freq > 200) {
    divider = 4;
    prescaler = TC_CLOCK_PRESCALER_DIV4;
    compare = (48000000/4)/freq;
  } else if (freq > 100) {
    divider = 8;
    prescaler = TC_CLOCK_PRESCALER_DIV8;
    compare = (48000000/8)/freq;
  } else if (freq > 50) {
    divider = 16;
    prescaler = TC_CLOCK_PRESCALER_DIV16;
    compare = (48000000/16)/freq;
  } else if (freq > 12) {
    divider = 64;
    prescaler = TC_CLOCK_PRESCALER_DIV64;
    compare = (48000000/64)/freq;
  } else if (freq > 3) {
    divider = 256;
    prescaler = TC_CLOCK_PRESCALER_DIV256;
    compare = (48000000/256)/freq;
  } else if (freq >= 0.75) {
    divider = 1024;
    prescaler = TC_CLOCK_PRESCALER_DIV1024;
    compare = (48000000/1024)/freq;
  } else {
    Serial.println("Invalid frequency");
    while (1) delay(10);
  }
  zerotimer.enable(false);
  zerotimer.configure(prescaler,       // prescaler
          TC_COUNTER_SIZE_16BIT,       // bit width of timer/counter
          TC_WAVE_GENERATION_MATCH_PWM // frequency or PWM mode
          );

  zerotimer.setCompare(0, compare);
  zerotimer.setCallback(true, TC_CALLBACK_CC_CHANNEL0, TimerCallback0);
  zerotimer.enable(true);
  
}

void setup() {
  matrix.begin();
  matrix.setTextWrap(false);
  matrix.setBrightness(255);
  matrix.fillScreen(0);
  //matrix.setFont(&Capefont);
  matrix.show();
  matrix.setRemapFunction(myRemapFn);
  randomSeed(analogRead(0));
  randomizeWorld();
  collarColor = random(255);

  // Define buttons
  pinMode(ledButton, INPUT);
  pinMode(heatingButton, INPUT);
  pinMode(heaterPin, OUTPUT);
  pinMode(heaterLedPin, OUTPUT);
  Serial.begin(57600);
  Serial.println("Starting Cape of Life...");
  
  // enable timer overflow interrupt for both Timer0 and Timer1
  // initialize timer1 
  noInterrupts();           // disable all interrupts
  setupTimer();
  interrupts();             // enable all interrupts
}

uint16_t myRemapFn(uint16_t x, uint16_t y) {
  uint16_t newCoords;
  uint16_t mLength = SIZEY;
  if(y >= SIZEY){
//    Serial.print("x:");
//    Serial.print(x);
//    Serial.print(" y:");
//    Serial.println(y);
//    Serial.println(SIZEY+COLLAR_OFFSET/SIZEX);
    return (2*x) + y-SIZEY;
  } else if(x%2 == 1) {
    newCoords = mLength*x + ((mLength-1) - y + COLLAR_OFFSET);
  } else {
    newCoords = mLength*x + (y+COLLAR_OFFSET);
  }
  return newCoords;
}

void randomizeWorld() {
  for (int i = 0; i < SIZEX; i++) {
    for (int j = 0; j < SIZEY; j++) {
      if (random(100) < density) {
        world[i][j][0] = 1;
      }
      else {
        world[i][j][0] = 0;
      }
      // Uncomment for manual control
      //world[i][j][0] = 0;

      
      world[i][j][1] = 0;
    }
  }
  
  // Glider for calibration
  /*world[1][3][0] = 1;
  world[2][1][0] = 1;
  world[2][3][0] = 1;
  world[3][2][0] = 1;
  world[3][3][0] = 1;*/

  oldColor = random(255);
}
 
void loop() {
  ledModeSwitch();
}

const uint16_t colors[] = { matrix.Color(255, 255, 255), matrix.Color(0, 255, 0), matrix.Color(0, 0, 255) };
int x    = matrix.width();
int pass = 0;

void ledModeSwitch() {
  Serial.print("LedMode: ");
  Serial.println(ledMode);
  switch(ledMode) {
    case 0:
    colorWipe(matrix.Color(0, 0, 0), 1); // Green
    break;
    
    case 1:
    theaterChaseRainbow(50);
    break;

    case 2:
    gameOfLifeLoop();
    break;

    case 3:
    gPal = HeatColors_p;
    Fire2012WithPalette();
    break;

    case 4:
    gPal = InverseHeatColors_p;
    Fire2012WithPalette();
    break;

    case 5:
    rainbowCycle(100);
    break;
    
    default:
    ledMode = 0;
  }
  ledModeInterrupted = false;
}

void colorWipe(uint32_t c, uint8_t wait) {
  
  for(uint16_t i=0; i<MATRIX_DIMENSIONS; i++) {
      matrix.drawPixel(i%SIZEX, i/SIZEX, c);
      matrix.show();
      delay(wait);
  }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;
  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    if(!ledModeInterrupted) {
      for(i=0; i< MATRIX_DIMENSIONS; i++) {
        matrix.drawPixel(i%SIZEX, i/SIZEX, Wheel(((i * 256 / matrix.numPixels()) + j) & 255));
      }
      matrix.show();
      delay(wait);
    }
  }
}

void gameOfLifeLoop() {
    // Some example procedures showing how to display to the pixels:
  uint16_t i, j, k;
  byte newColor = oldColor + 4;
  LifeWheel(oldColor & 255, rgbOld);
  LifeWheel(newColor & 255, rgbNew);
  oldColor = newColor;
  // Fade out old generation
  for(i = 0; i<COLORCYCLEAMOUNT; i++) {
    if(!ledModeInterrupted) {
      for (j = 0; j < SIZEX; j++) {
        for (k = 0; k < SIZEY; k++) {
          if (world[j][k][2] && !world[j][k][0]) {
            setGridPixelColor(j, k, matrix.Color(rgbOld[0] - (rgbOld[0] * i) / COLORCYCLEAMOUNT, rgbOld[1] - (rgbOld[1] * i) / COLORCYCLEAMOUNT, rgbOld[2] - (rgbOld[2] * i) / COLORCYCLEAMOUNT));
          } else {

          }
        }
      }
    }
  }
  // Display current generation
  for(i = 0; i<COLORCYCLEAMOUNT; i++) {
    if(!ledModeInterrupted) {
      for (j = 0; j < SIZEX; j++) {
        for (k = 0; k < SIZEY; k++) {
          if (world[j][k][0] && !world[j][k][2]) {
            setGridPixelColor(j, k, matrix.Color((i*rgbNew[0])/COLORCYCLEAMOUNT, (i*rgbNew[1])/COLORCYCLEAMOUNT, (i*rgbNew[2])/COLORCYCLEAMOUNT));
          } else if( world[j][k][0] && world[j][k][2]){
            setGridPixelColor(j, k, matrix.Color(rgbNew[0], rgbNew[1], rgbNew[2]));
          } else { 
            setGridPixelColor(j, k, matrix.Color(0, 0, 0));
          }
        }
      }
    }
  }
  delay(DELAY);
  
  // Birth and death cycle
  for (int x = 0; x < SIZEX; x++) {
    for (int y = 0; y < SIZEY; y++) {
      // Default is for cell to stay the same
      world[x][y][1] = world[x][y][0];
      int count = neighbours(x, y);
      if (count == 3 && world[x][y][0] == 0) {
        // A new cell is born
        world[x][y][1] = 1;
      }
      if ((count < 2 || count > 3) && world[x][y][0] == 1) {
        // Cell dies
        world[x][y][1] = 0;
      }
    }
  }

  int sameCells = 0;
  // Copy next generation into place
  for (int x = 0; x < SIZEX; x++) {
    for (int y = 0; y < SIZEY; y++) {
      if ( world[x][y][2] == world[x][y][1] ) {
        sameCells++;
      }
      world[x][y][2] = world[x][y][0];
      world[x][y][0] = world[x][y][1];
      // Check if we have hit stagnation by comparing world n-1 to world n+1 
    }
  }
  // We have hit stagnation
  if(sameCells == SIZEX * SIZEY) {
    randomizeWorld(); 
  }
}
 
int neighbours(int x, int y) {
  
  // Upper left, upper right, lower left, lower right, left, right, down, up.
  if (x == 0 && y == 0) {
    return world[(x + 1)][y][0] + world[(x + 1)][(y + 1)][0] + 
        0 + 0 + 
        world[x][(y + 1)][0] + 0 + 
        0 + 0;
  } else if (x > SIZEX && y == 0) {
    return  0 + 0 + 
        world[(x - 1)][y][0] + world[(x - 1)][(y + 1)][0] + 
        world[x][(y + 1)][0] + 0 + 
        0 + 0;
  } else if (x == 0 && y == SIZEY) {
    return  world[(x + 1)][y][0] + 0 + 
        0 + 0 + 
        0 + world[(x + 1)][(y - 1)][0] + 
        world[x][(y - 1)][0] + 0;
  } else if (x == SIZEX && y == SIZEY) {
    return  0 + 0 + 
        world[(x - 1)][y][0] + 0 + 
        0 + 0 + 
        world[x][(y - 1)][0] + world[(x - 1)][(y - 1)][0];
  } else if (x == 0) {
    return  world[(x + 1)][y][0] + world[(x + 1)][(y + 1)][0] + 
        0 + 0 + 
        world[x][(y + 1)][0] + world[(x + 1)][(y - 1)][0] + 
        world[x][(y - 1)][0] + 0;
  } else if (x == SIZEX) {
    return  0 + 0 + 
        world[(x - 1)][y][0] + world[(x - 1)][(y + 1)][0] +  
        world[x][(y + 1)][0] + 0 + 
        world[x][(y - 1)][0] + world[(x - 1)][(y - 1)][0];
  } else if (y == 0) {
    return  world[(x + 1)][y][0] + world[(x + 1)][(y + 1)][0] +
         world[(x - 1)][y][0] + world[(x - 1)][(y + 1)][0] +
         world[x][(y + 1)][0] + 0 +
         0 + 0;
  } else if (y == SIZEY) {
    return  world[(x + 1)][y][0] + 0 +
         world[(x - 1)][y][0] + 0 +
         0 + world[(x + 1)][(y - 1)][0] +
         world[x][(y - 1)][0] + world[(x - 1)][(y - 1)][0];
  } else {
    return  world[(x + 1)][y][0] + world[(x + 1)][(y + 1)][0] +
           world[(x - 1)][y][0] + world[(x - 1)][(y + 1)][0] +
           world[x][(y + 1)][0] + world[(x + 1)][(y - 1)][0] +
           world[x][(y - 1)][0] + world[(x - 1)][(y - 1)][0];
  }
}
 
// Convert the pixel coordinates into the actual pixel and display
void setGridPixelColor(int x, int y, uint16_t c) {
  matrix.drawPixel(x, y, c);
  matrix.show();
  //delay(1000);
}
 
// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
void LifeWheel(byte LifeWheelPos, byte rgb[]) {
  if(LifeWheelPos < 85) {
    rgb[0] = LifeWheelPos * 3;
    rgb[1] = 255 - LifeWheelPos * 3;
    rgb[2] = 0;
  } else if(LifeWheelPos < 170) {
   LifeWheelPos -= 85;
   rgb[0] = 255 - LifeWheelPos * 3;
   rgb[1] = 0;
   rgb[2] = LifeWheelPos * 3;
  } else {
   LifeWheelPos -= 170;
   rgb[0] = 0;
   rgb[1] = LifeWheelPos * 3;
   rgb[2] = 255 - LifeWheelPos * 3;
  }
}

uint32_t Wheel(byte WheelPos) {
  if(WheelPos < 85) {
   return matrix.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if(WheelPos < 170) {
   WheelPos -= 85;
   return matrix.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
   WheelPos -= 170;
   return matrix.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}


//Theatre-style crawling lights with rainbow effect
void theaterChaseRainbow(uint8_t wait) {
  for (int j=0; j < 256; j++) {     // cycle all 256 colors in the wheel
    if (!ledModeInterrupted) {
      for (int q=0; q < 3; q++) {
        for(uint16_t i=0; i < MATRIX_DIMENSIONS; i=i+4) {
            matrix.drawPixel(i/SIZEY, i%SIZEY+q, Wheel( (i+j) % 255));
        }
        matrix.show();
        delay(wait);
  
        for(uint16_t i=0; i < MATRIX_DIMENSIONS; i=i+4) {
            matrix.drawPixel(i/SIZEY, i%SIZEY+q, 0);
        }
      }
      
    }
  }
}


// COOLING: How much does the air cool as it rises?
// Less cooling = taller flames.  More cooling = shorter flames.
// Default 55, suggested range 20-100 
int COOLING = 40;

// SPARKING: What chance (out of 255) is there that a new spark will be lit?
// Higher chance = more roaring fire.  Lower chance = more flickery fire.
// Default 120, suggested range 50-200.
int SPARKING = 40;

#define NUM_LEDS SIZEY

#define NUM_STRIPS 10
#define STRIP_INCREMENT 1

#define SAMPLE_WINDOW   10  // Sample window for average level
#define INPUT_FLOOR     56  // Lower range of mic sensitivity in dB SPL
#define INPUT_CEILING  110  // Upper range of mic sensitivity in db SPL

void Fire2012WithPalette()
{
// Array of temperature readings at each simulation cell
  static byte heat[NUM_LEDS];
  float peakToPeak = 0;   // peak-to-peak level
  //get peak sound pressure level over the sample window
  peakToPeak = CircuitPlayground.mic.soundPressureLevel(SAMPLE_WINDOW);

  //limit to the floor value
  peakToPeak = max(INPUT_FLOOR, peakToPeak);
  Serial.println(peakToPeak);

  if (ledMode == 5) {
    SPARKING = (peakToPeak - 30)*2;
    COOLING = 115 - (peakToPeak);
    Serial.print("Sparking :");
    Serial.println(SPARKING);
    Serial.print("Cooling :");
    Serial.println(COOLING);
  } else {
    SPARKING = (peakToPeak - 50)*2;
    COOLING = 125 - (peakToPeak);
  }

  

  for (int x_strip = 0; x_strip < NUM_STRIPS; x_strip = x_strip + STRIP_INCREMENT) {
    random16_add_entropy( random());
  // Step 1.  Cool down every cell a little
    for( int i = 0; i < NUM_LEDS; i++) {
      heat[i] = qsub8( heat[i],  random8(0, ((COOLING * 10) / NUM_LEDS) + 2));
    }
  
    // Step 2.  Heat from each cell drifts 'up' and diffuses a little
    for( int k= NUM_LEDS - 1; k >= 2; k--) {
      heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2] ) / 3;
    }
    
    // Step 3.  Randomly ignite new 'sparks' of heat near the bottom
    if( random8() < SPARKING ) {
      int y = random8(7);
      heat[y] = qadd8( heat[y], random8(160,255) );
    }

    // Step 4.  Map from heat cells to LED colors
    for( int j = 0; j < NUM_LEDS; j++) {
      // Scale the heat value from 0-255 down to 0-240
      // for best results with color palettes.
      byte colorindex = scale8( heat[j], 240);
      CRGB color = ColorFromPalette( gPal, colorindex);
      int pixelnumber;
      if( gReverseDirection ) {
        pixelnumber = (NUM_LEDS-1) - j;
      } else {
        pixelnumber = j;
      }
      matrix.drawPixel(x_strip, pixelnumber, matrix.Color(color.red, color.green, color.blue));
    } 
  }
  matrix.show();
  delay(32);
}
