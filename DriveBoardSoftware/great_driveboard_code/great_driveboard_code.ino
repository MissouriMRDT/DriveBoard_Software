#include "RoveComm.h"
#include "RoveWatchdog.h"

#define FR_MOTOR PF_3
#define FL_MOTOR PG_0
#define MR_MOTOR PL_5
#define ML_MOTOR PL_0
#define RR_MOTOR PL_2
#define RL_MOTOR PL_3

RoveCommEthernet RoveComm;
rovecomm_packet packet;
RoveWatchdog Watchdog;

int16_t rcleftspeed;
int16_t rcrightspeed;
uint8_t rightspeed;
uint8_t leftspeed;

const byte DRIVE_ZERO        = 127;


uint8_t motorButtons[6] = {FR_MOTOR, MR_MOTOR, RR_MOTOR, FL_MOTOR, ML_MOTOR, RL_MOTOR};
uint8_t motorSpeeds[6] = {DRIVE_ZERO, DRIVE_ZERO, DRIVE_ZERO, DRIVE_ZERO, DRIVE_ZERO, DRIVE_ZERO}; //FR, MR, RR, FL, ML, RL


void EStop();

void setup() {
  // initialize both serial ports:
  Serial.begin(19200);
  Serial2.begin(19200);
  Serial3.begin(19200);
  Serial4.begin(19200);
  Serial5.begin(19200);
  Serial6.begin(19200);
  Serial7.begin(19200);
  RoveComm.begin(RC_DRIVEBOARD_FOURTHOCTET, RC_ROVECOMM_ETHERNET_DRIVE_LIGHTING_BOARD_PORT);
  Watchdog.begin(Estop, 150); 
   for(int i = 0; i < 6; i++)
  {
    pinMode(motorButtons[i], INPUT);
  }
}

void loop() {
  // read from port 1, send to port 0:
    //rcrightspeed = _____ ;
    //rcleftspeed = _____ ;
    packet = RoveComm.read();
    Serial.println(packet.data_id);
  
    switch(packet.data_id)
    {
      case RC_DRIVEBOARD_DRIVELEFTRIGHT_DATAID:
        Serial.println("left/right drive command recieved");
        int16_t *leftrightspeeds;
        leftrightspeeds = (int16_t*)packet.data;

        rcleftspeed = leftrightspeeds[0];
        rcrightspeed = leftrightspeeds[1];
        rcleftspeed = -rcleftspeed;

        Serial.println(rcleftspeed);
        Serial.println(rcrightspeed);

        rightspeed = map(rcrightspeed,-1000,1000,0,255);
        leftspeed = map(rcleftspeed,-1000,1000,0,255);

        for(int i = 0; i < 3; i++)
        {
          motorSpeeds[i] = rightspeed;
        }
        for(int i = 3; i < 6; i++)
        {
          motorSpeeds[i] = leftspeed;
        }
        Watchdog.clear();
        break;
      case RC_DRIVEBOARD_DRIVEINDIVIDUAL_DATAID:
        Serial.println("individual drive command recieved");
        int16_t *individualrcspeeds;
        individualrcspeeds = (int16_t*)packet.data;
        Watchdog.clear();
        break;
    }
    for(int i = 0; i < 6; i++)
    {
      if(digitalRead(motorButtons[i]))
      {
        Serial.println("Button is pressed");
        motorSpeeds[i] = 140;
      }
    }
    Serial.println("Motor speeds: ");
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
