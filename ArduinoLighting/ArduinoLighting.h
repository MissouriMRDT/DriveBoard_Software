#ifndef _ArduinoLighting
#define _ArduinoLighting

#include <Adafruit_NeoPixel.h>
#include<SPI.h>
byte data; 

#define DELAYVAL 500 // Time (in milliseconds) to pause between pixels

Adafruit_NeoPixel pixels(256, 9, NEO_GRB);

#endif
