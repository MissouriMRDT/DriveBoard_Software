/*
 * DriveBoard Software Rev 2 2013
 * Used with ODrive Motor Controllers
 * Writes Serial to 3 motor drives
 * 
 * Andrew Van Horn
 */

#include "RoveComm.h"
#include "RovesODrive.h"
#include "DriveBoardSoftware.h"


void checkButtons()
{
    for(int i = 0; i<NUMMOTORS; i++)
    {
      if(digitalRead(motor[i].button_pin))
      {
        motor[i].speed = (digitalRead(DIRECTION_SWITCH_PIN)?1:-1)*BUTTONPRESS_SPEED;
      }
      else if(watchdog_triggered)
      {
        motor[i].speed = 0;
      }
    }
}

void writeSpeeds()
{
  for(int i = 0; i<NUMDRIVES; i++)
  {
    for(int j = 0; j<2; j++)
    {
      Drive[i].motor[j].setSpeed(motor[i+j].speed);
      Serial.print(i+j);
      Serial.print(":");
      Serial.println(motor[i+j].speed);
    }
  }
  
}

void getSpeeds()
{
  for(int i = 0; i<NUMDRIVES; i++)
  {
    for(int j = 0; j<2; j++)
    {
      Serial.print(i+j);
      Serial.print(":");
      Serial.println(Drive[i].motor[j].getSpeed());
    }
  }
}

void setup()
{
  Serial.begin(115200);

  pinMode(DIRECTION_SWITCH_PIN, INPUT);

  for(int i = 0; i<NUMMOTORS; i++)
  {
    pinMode(motor[i].button_pin, INPUT);
  }
                                
  pinMode(WATCHDOG_IND_PIN    , OUTPUT);
  pinMode(LSPEED_IND_PIN      , OUTPUT);
  pinMode(RSPEED_IND_PIN      , OUTPUT);

  for(int i = 0; i<NUMDRIVES; i++)
  {
    Drive[i].begin();
  }

    
}

void loop()
{
  checkButtons();
  delay(100);
  Serial.println(".");

  if(watchdog_triggered)
  {
    digitalWrite(WATCHDOG_IND_PIN, HIGH);
  }

  writeSpeeds();

  getSpeeds();
}