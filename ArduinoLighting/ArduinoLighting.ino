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
    pixels.setBrightness(30);
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
      pixels.clear(); // autonomous operation
      for(int i=0; i<256; i++) 
      { 
          //Setting color to solid red
          pixels.setPixelColor(i, pixels.Color(150, 0, 0));
      }
      pixels.show();
      break;
    case 1:
     pixels.clear(); // teleop drive
      for(int i=0; i<256; i++) 
      { 
          //Setting color to solid blue
          pixels.setPixelColor(i, pixels.Color(0, 0, 150));
      }
      pixels.show();
      break;
    case 2:
      pixels.clear(); // successful completion of leg
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
    case 3:
      pixels.clear(); // display belgium flag
      for(int i=0; i<80; i++) 
      { 
          //Setting color to green
          pixels.setPixelColor(i, pixels.Color(0, 0, 0));
      }
      for(int i=80; i<175; i++) 
      { 
          //Setting color to green
          pixels.setPixelColor(i, pixels.Color(252, 232, 10));
      }
      for(int i=176; i<256; i++) 
      { 
          //Setting color to green
          pixels.setPixelColor(i, pixels.Color(252, 10, 10));
      }
      pixels.show();
      break;        
 }
}
