/*
 Cape of Life
 Adapted from http://pastebin.com/f22bfe94d and Adafruit Neopixel tutorial.
 In this case; 0 is present, 1 is future, 2+ is past.
*/
 
#include <Adafruit_NeoPixel.h>
#include <avr/interrupt.h>
#include <avr/io.h>

// Columns were alternated between running up the cape and down the cape for wiring purposes. With this on, every other row will have the output flipped.
#define ALTERNATE_COLUMNS 1
#define DELAY 0
#define SIZEX 20
#define SIZEY 7
#define OFFSET 10
// This determines how 'smooth' transitions between worlds will be
#define COLORCYCLEAMOUNT 16

const int ledButton = 9;    // the number of the pushbutton pin
const int heatingButton = 10;    // the number of the pushbutton pin
const int heaterPin = 6;    // the number of the pushbutton pin
const int heaterLedPin = 7;
int heaterLedState = LOW;         // the current state of the output pin
int ledButtonState;             // the current reading from the input pin
int heaterButtonState;             // the current reading from the input pin
int lastLedButtonState = LOW;   // the previous reading from the input pin
int lastHeaterButtonState = LOW;   // the previous reading from the input pin
long lastDebounceTime = 0;  // the last time the output pin was toggled
long debounceDelay = 50;    // the debounce time; increase if the output flickers

byte ledMode = 0;
byte heaterMode = 0;
byte collarLed = 0;
byte collarColor = 0;

byte world[SIZEX][SIZEY][3], oldColor, rgbOld[3], rgbNew[3];
boolean firstCycle = true;
boolean ledModeInterrupted = false;
long density = 22;
 
// Parameter 1 = number of pixels in strip
// Parameter 2 = pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_RGB     Pixels are wired for RGB bitstream
//   NEO_GRB     Pixels are wired for GRB bitstream
//   NEO_KHZ400  400 KHz bitstream (e.g. FLORA pixels)
//   NEO_KHZ800  800 KHz bitstream (e.g. High Density LED strip)
Adafruit_NeoPixel strip = Adafruit_NeoPixel(150, 12, NEO_GRB + NEO_KHZ800);
 
void setup() {
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
  randomSeed(analogRead(0));
  randomizeWorld();
  collarColor = random(255);
  
  // Define buttons
  pinMode(ledButton, INPUT);
  pinMode(heatingButton, INPUT);
  pinMode(heaterPin, OUTPUT);
  pinMode(heaterLedPin, OUTPUT);
  Serial.begin(9600);
  Serial.println("Starting Cape of Life...");
  
  // enable timer overflow interrupt for both Timer0 and Timer1
  // initialize timer1 
  noInterrupts();           // disable all interrupts
  TCCR3A = 0;
  TCCR3B = 0;
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT3  = 0;
  TCNT1  = 0;
  
  OCR3A = 31250;            // compare match register 16MHz/256/2Hz
  TCCR3B |= (1 << WGM12);   // CTC mode
  TCCR3B |= (0 << CS30);    // 256 prescaler
  TCCR3B |= (0 << CS31);    // 256 prescaler
  TCCR3B |= (1 << CS32);    // 256 prescaler 
  TIMSK3 |= (1 << OCIE3A);  // enable timer compare interrupt

  OCR1A = 1563;            // compare match register 16MHz/256/2Hz
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= (0 << CS10);    // 256 prescaler
  TCCR1B |= (0 << CS11);    // 256 prescaler
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
  interrupts();             // enable all interrupts
}

ISR(TIMER1_COMPA_vect)          // timer compare interrupt service routine
{
  int reading = digitalRead(ledButton);
  int reading2 = digitalRead(heatingButton);
  if (reading != lastLedButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  } 
  
  if (reading2 != lastHeaterButtonState) {
    lastDebounceTime = millis();

  }
  
  if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:
    // if the button state has changed:
    if (reading != ledButtonState) {
      ledButtonState = reading;

      // only toggle the LED if the new button state is HIGH
      if (ledButtonState == HIGH) {
        ledMode++;
        Serial.println("Changing LED mode!");
        ledModeInterrupted = true;
      }
    }
    if (reading2 != heaterButtonState) {
      heaterButtonState = reading2;
      
      if (heaterButtonState == HIGH) {
        heaterMode++;
        switch(heaterMode) {
          case 0:
          Serial.println("Heater off!");
          digitalWrite(heaterLedPin, LOW);
          break;
          
          case 1:
          Serial.println("Heater low!");
          analogWrite(heaterPin, 55);
          digitalWrite(heaterLedPin, HIGH);
          break;
          
          case 2:
          Serial.println("Heater med!");
          analogWrite(heaterPin, 110);
          digitalWrite(heaterLedPin, HIGH);
          break;
          
          case 3:
          Serial.println("Heater hi!");
          analogWrite(heaterPin, 220);
          digitalWrite(heaterLedPin, HIGH);
          break;
          
          default:
          heaterMode = 0;
          Serial.println("Heater off!");
          
        }
      }
    }
  }
  lastLedButtonState = reading;
  lastHeaterButtonState = reading2;
}

// Collar Control

ISR(TIMER3_COMPA_vect) {
  strip.setPixelColor(collarLed, Wheel(collarColor));
  collarLed++;
  if(collarLed >= 10) {
    collarLed = 0;
    collarColor = random(255);
  }
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
      //world[i][j][0] = 0;
      world[i][j][1] = 0;
    }
  }
  world[1][5][0] = 1;
  world[2][4][0] = 1;
  world[2][5][0] = 1;
  world[3][4][0] = 1;
  world[3][5][0] = 1;
  world[4][4][0] = 1;
  oldColor = random(255);
}
 
void loop() {
  ledModeSwitch();
}

void ledModeSwitch() {
  Serial.print("LedMode: ");
  Serial.println(ledMode);
  switch(ledMode) {
    case 0:
    gameOfLifeLoop();
    break;
    
    case 1:
    rainbowCycle(10);
    break;
    
    case 2:
    colorWipe(strip.Color(0, 0, 0), 5); // Green
    break;
    
    default:
    ledMode = 0;
    gameOfLifeLoop();
  }
  ledModeInterrupted = false;
}

void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=10; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, c);
      strip.show();
      delay(wait);
  }
}

// Slightly different, this makes the rainbow equally distributed throughout
void rainbowCycle(uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    if(!ledModeInterrupted) {
      for(i=10; i< strip.numPixels(); i++) {
        strip.setPixelColor(i, Wheel(((i * 256 / strip.numPixels()) + j) & 255));
      }
      strip.show();
      delay(wait);
    }
  }
}

void gameOfLifeLoop() {
    // Some example procedures showing how to display to the pixels:
  //colorWipe(strip.Color(255, 0, 0), 50); // Red
  //colorWipe(strip.Color(0, 255, 0), 50); // Green
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
            setGridPixelColor(j, k, strip.Color(rgbOld[0] - (rgbOld[0] * i) / COLORCYCLEAMOUNT, rgbOld[1] - (rgbOld[1] * i) / COLORCYCLEAMOUNT, rgbOld[2] - (rgbOld[2] * i) / COLORCYCLEAMOUNT));
          } else {
            //setGridPixelColor(j, k, strip.Color(0, 0, 0));
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
            setGridPixelColor(j, k, strip.Color((i*rgbNew[0])/COLORCYCLEAMOUNT, (i*rgbNew[1])/COLORCYCLEAMOUNT, (i*rgbNew[2])/COLORCYCLEAMOUNT));
          } else if( world[j][k][0] && world[j][k][2]){
            setGridPixelColor(j, k, strip.Color(rgbNew[0], rgbNew[1], rgbNew[2]));
          } else { 
            setGridPixelColor(j, k, strip.Color(0, 0, 0));
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
void setGridPixelColor(int x, int y, uint32_t c) {
  uint32_t pixel;
  if(ALTERNATE_COLUMNS && x%2 == 1)
  {
    pixel = (x*10) + (9-y);
  } else {
    pixel = (x*10) + y;
  }
  if(x < SIZEX && y < SIZEY) {
    strip.setPixelColor(pixel + OFFSET, c);
  }
  strip.show();
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
   return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if(WheelPos < 170) {
   WheelPos -= 85;
   return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
   WheelPos -= 170;
   return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}
