#include "DriveBoardSoftware.h"

void setup() {
  // initialize all serial ports:
  Serial.begin(19200);
  Serial2.begin(19200);
  Serial3.begin(19200);
  Serial4.begin(19200);
  Serial5.begin(19200);
  Serial6.begin(19200);
  Serial7.begin(19200);

  RoveComm.begin(RC_DRIVEBOARD_FOURTHOCTET, RC_ROVECOMM_ETHERNET_DRIVE_LIGHTING_BOARD_PORT);
  
  //after 150 seconds of no comms, disable drive
  Watchdog.begin(Estop, 150); 


  //set up SPI communication
  SPI.setModule(2);
  SPI.setClockDivider(SPI_CLOCK_DIV8);

  //enable communication with slave
  pinMode(SLAVE_SELECT, OUTPUT);
  digitalWrite(SLAVE_SELECT, LOW); 

  //set buttons to input
  for(int i = 0; i < 6; i++)
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

        for(int i = 0; i < 3; i++)
        {
          motorSpeeds[i] = leftspeed;
        }
        for(int i = 3; i < 6; i++)
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
        //send the Nano the new LED state
        SPI.transfer(state[0]);
        break;
    }

    //check for button presses and override speeds if so
    for(int i = 0; i < 6; i++)
    {
      if(digitalRead(motorButtons[i]))
      {
        motorSpeeds[i] = 140;
      }
    }
    for(int i = 0; i < 6; i++)
    {
      Serial.println(motorSpeeds[i]);
    }
    Serial2.write(motorSpeeds[0]); //Rear
    Serial3.write(motorSpeeds[1]); //Middle
    Serial4.write(motorSpeeds[2]); //Front
    Serial5.write(motorSpeeds[3]); //Front
    Serial6.write(motorSpeeds[4]); // Middle
    Serial7.write(motorSpeeds[5]); //Rear
}

void Estop()
{
    for(int i = 0; i < 6; i++)
    {
      if(!digitalRead(motorButtons[i]))
      {
        motorSpeeds[i] = DRIVE_ZERO;
      }
    }
    Serial.println("Watchdog cleared");
    Watchdog.clear();
}
