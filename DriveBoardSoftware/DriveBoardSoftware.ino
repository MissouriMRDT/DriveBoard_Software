//DriveBoard Software Rev 1 2021

#include "DriveBoardSoftware.h"

EthernetServer TCPServer(RC_ROVECOMM_ETHERNET_DRIVE_LIGHTING_BOARD_PORT);

void setup() {
  // initialize all serial ports:
  Serial.begin(19200);
  Serial2.begin(19200);
  Serial3.begin(19200);
  Serial4.begin(19200);
  Serial5.begin(19200);

  RoveComm.begin(RC_DRIVEBOARD_FOURTHOCTET, &TCPServer);
  
  //after 150 seconds of no comms, disable drive
  Watchdog.begin(Estop, 150); 

  //set buttons to input
  for(int i = 0; i < 4; i++)
  {
    pinMode(motorButtons[i], INPUT);
  }
}

void loop() {
    packet = RoveComm.read();
  
    switch(packet.data_id)
    {
      //read in a left and right speed
      case RC_DRIVEBOARD_DRIVELEFTRIGHT_DATAID:
        int16_t *leftrightspeeds;
        leftrightspeeds = (int16_t*)packet.data;

        rightspeed = map(leftrightspeeds[0],-1000,1000,0,255);
        leftspeed = map(-leftrightspeeds[1],-1000,1000,0,255);

        for(int i = 0; i < 2; i++)
        {
          motorSpeeds[i] = leftspeed;
        }
        for(int i = 2; i < 4; i++)
        {
          motorSpeeds[i] = rightspeed;
        }
        Watchdog.clear();
        break;
      //read in individual speeds
      case RC_DRIVEBOARD_DRIVEINDIVIDUAL_DATAID:
        int16_t *individualrcspeeds;
        individualrcspeeds = (int16_t*)packet.data;
        Watchdog.clear();
        break;
      case RC_LIGHTINGBOARD_STATE_DISPLAY_DATAID:
        uint8_t* state = (uint8_t*)packet.data;
        Serial.println("Writing new lighting state");
        break;
    }

    //check for button presses and override speeds if so
    for(int i = 0; i < 4; i++)
    {
      if(digitalRead(motorButtons[i]))
      {
        motorSpeeds[i] = 140;
      }
    }
    for(int i = 0; i < 4; i++)
    {
      Serial.println(motorSpeeds[i]);
    }
    Serial2.write(motorSpeeds[0]); //FR
    Serial3.write(motorSpeeds[1]); //BR
    Serial4.write(motorSpeeds[2]); //FL
    Serial5.write(motorSpeeds[3]); //BL
}

void Estop()
{
    for(int i = 0; i < 4; i++)
    {
      if(!digitalRead(motorButtons[i]))
      {
        motorSpeeds[i] = DRIVE_ZERO;
      }
    }
    Serial.println("Watchdog cleared");
    Watchdog.clear();
}
