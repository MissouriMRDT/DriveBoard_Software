#include "RoveComm.h"
#include "RoveWatchdog.h"

RoveCommEthernet RoveComm;
rovecomm_packet packet;
RoveWatchdog Watchdog;

int16_t rcleftspeed;
int16_t rcrightspeed;
uint8_t rightspeed;
uint8_t leftspeed;

const byte DRIVE_ZERO        = 127;

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

        rightspeed = map(rcrightspeed,-1000,1000,0,255);
        leftspeed = map(rcleftspeed,-1000,1000,0,255);
        Watchdog.clear();
        break;
      case RC_DRIVEBOARD_DRIVEINDIVIDUAL_DATAID:
        Serial.println("individual drive command recieved");
        int16_t *individualrcspeeds;
        individualrcspeeds = (int16_t*)packet.data;
        Watchdog.clear();
        break;
    }
    Serial.println(leftspeed);
    Serial.println(rightspeed);
    Serial2.write(rightspeed); //Rear
    Serial3.write(rightspeed); //Middle
    Serial4.write(rightspeed); //Front
    Serial5.write(leftspeed); //Front
    Serial6.write(leftspeed); // Middle
    Serial7.write(leftspeed); //Rear
    
  
}

void Estop()
{
    rightspeed = DRIVE_ZERO;
    leftspeed = DRIVE_ZERO;
    Serial.println("Watchdog cleared");
    Watchdog.clear();
}
