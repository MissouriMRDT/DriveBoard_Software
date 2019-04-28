/*
 * DriveBoard Software Rev 2 2018
 * Used with DriveBoard Rev 2 2018
 * Writes Serial to 6 motor controllers, controls RGB LD strips, Headlights, and 4 Dropbay Servos
 * 
 * Andrew Van Horn, Judah Schad
 * Disclaimer- I know this code is jank. The servo libraries were giving me trouble at competition, and this conficguration somehow worked. Give me a break
 */

#include "RoveComm.h"
#include "RoveWatchdog.h"

//////////////////////////////////////////////////
// We send serial speed bytes to motor controllers 
const byte DRIVE_MAX_FORWARD = 255;
const byte DRIVE_MAX_REVERSE = 0;
const byte DRIVE_ZERO        = 127;
byte left_drive_speed        = DRIVE_ZERO;
byte right_drive_speed       = DRIVE_ZERO;

struct motor
{
  int16_t roveCommSpeed;
  byte serialSpeed;
}

motor Motor[6];

RoveCommEthernetUdp RoveComm;
RoveWatchdog        Watchdog;

void roveEstopDriveMotors();

/////////////////////////////////////////////////////////////////////////////////////////////////

void setup()
{
  Serial.begin( 9600); 
  Serial2.begin(19200); 
  Serial3.begin(19200);
  Serial4.begin(19200);
  Serial5.begin(19200);
  Serial6.begin(19200);
  Serial7.begin(19200);
  delay(1);
  
  RoveComm.begin(RC_DRIVEBOARD_FOURTHOCTET); 
  delay(1);

  Watchdog.begin(roveEstopDriveMotors, 150); 
  delay(1);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop()
{
   

  /*
  uint16_t data_id   = 0;
  size_t   data_size = 0;
  uint8_t  data_value[4];
  roveComm_GetMsg(&data_id, &data_size, &data_value);
  */
  rovecomm_packet packet;
  packet = RoveComm.read();
  if(packet.data_id != 0)
  {
    Serial.print("packet.data_id : ");
    Serial.println(packet.data_id);
  }
  if(packet.data_id != 0)
  {
   // Serial.println("************************************");
    switch (packet.data_id) 
    {     
      case RC_DRIVEBOARD_DRIVELEFTRIGHT_DATAID:
      {         
        int16_t left_speed_temp  =  -packet.data[0]; // 2 high bytes contain RED's left speed  as int16
        int16_t right_speed_temp =  packet.data[1];        // 2 low  bytes contain RED's right speed as int16
  
        Serial.print("RoveComm Speed:"); Serial.print(left_speed_temp); Serial.print(","); Serial.println(right_speed_temp);

        Motor[0].roveCommSpeed = left_speed_temp;
        Motor[1].roveCommSpeed = left_speed_temp;
        Motor[2].roveCommSpeed = left_speed_temp;
        Motor[3].roveCommSpeed = right_speed_temp;
        Motor[4].roveCommSpeed = right_speed_temp;
        Motor[5].roveCommSpeed = right_speed_temp;
        
        Watchdog.clear();
        break;  
      }
      case RC_DRIVEBOARD_DRIVEMOTORS_DATAID:
      {         
        for(int i = 0; i<6; i++)
        {
          Motor[0].roveCommSpeed = packet.data[i];
        }
        Watchdog.clear();
        break;  
      }
    }
  }

  for(int i = 0; i<6; i++)
  {
    Motor[i].serialSpeed = map(Motor[i].roveCommSpeed, RC_DRIVEBOARD_DRIVELEFTRIGHT_DRIVEMAXREVERSE, RC_DRIVEBOARD_DRIVELEFTRIGHT_DRIVEMAXFORWARD, DRIVE_MAX_REVERSE, DRIVE_MAX_FORWARD);
  }
  
  Serial2.write(Motor[0].serialSpeed);  //LF
  Serial3.write(Motor[1].serialSpeed ); //LM
  Serial4.write(Motor[2].serialSpeed ); //LR
  Serial5.write(Motor[3].serialSpeed ); //RF
  Serial6.write(Motor[4].serialSpeed);  //RM
  Serial7.write(Motor[5].serialSpeed);  //RR
  delay(1);
}

////////////////////////////////

void roveEstopDriveMotors() 
{ 
  left_drive_speed  = DRIVE_ZERO;
  right_drive_speed = DRIVE_ZERO;
  Watchdog.clear();
  
  Serial.println("Watchog Triggered");
  uint8_t b_watchdog_triggered = 1;    //Had to send data with a type
  RoveComm.write(RC_DRIVEBOARD_WACHDOGTRIGGERED_HEADER, b_watchdog_triggered);
}
