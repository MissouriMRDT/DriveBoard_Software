#include <SPI.h>
#include "ArduinoLighting.h"

void setup()
{
    Serial.begin(115200);
    Serial.println("Started");
    //sending data to master in
    pinMode(MISO,OUTPUT);

    //setting SPI module to slave
    SPCR |= _BV(SPE);

    //temporary hack
    data = 0;

    //attach interrupt so that we can grab incoming bytes
    SPI.attachInterrupt(); 

    pixels.begin();
}

ISR (SPI_STC_vect)
{
    Serial.println("Interrupt fired");
    data = SPDR;
    Serial.println(data);
}

void loop()
{
  switch(data)
  {
    case 0:
      pixels.clear(); 
      for(int i=0; i<256; i++) 
      { 
          //Setting color to solid blue
          pixels.setPixelColor(i, pixels.Color(0, 0, 150));
      }
      pixels.show();
      break;
    case 1:
      pixels.clear(); // Set all pixel colors to 'off'
      for(int i=0; i<256; i++) 
      { 
          //Setting color to solid red
          pixels.setPixelColor(i, pixels.Color(150, 0, 0));
      }
      pixels.show();
      break;
    case 2:
      pixels.clear(); // Set all pixel colors to 'off'
      pixels.show();
      delay(500);
      for(int i=0; i<256; i++) 
      { 
          //Setting color to green
          pixels.setPixelColor(i, pixels.Color(0, 150, 0));
      }
      pixels.show();
      delay(500);
      break;        
 }
}
