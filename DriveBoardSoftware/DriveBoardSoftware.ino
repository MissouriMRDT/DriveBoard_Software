/*
 * DriveBoard Software Rev 2 2018
 * Used with DriveBoard Rev 2 2018
 * Writes Serial to 6 motor controllers, controls RGB LD strips, Headlights, and 4 Dropbay Servos
 * 
 * Andrew Van Horn, Judah Schad
 * Disclaimer- I know this code is jank. The servo libraries were giving me trouble at competition, and this configuration somehow worked. Give me a break
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Energia
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

// RoveWare
//#include <RoveBoard.h>
//#include <RoveEthernet.h>
#include <RoveComm.h>


#include "RoveComm.h"
#include "Servo.h"
#include "SPI.h"



//////////////Inputs///////////////////////
#define DIRECTION_SW        PD_2
#define M1_SW               PK_2 //Left front
#define M2_SW               PK_3 //Left middle
#define M3_SW               PQ_0 //Left back
#define M4_SW               PP_4 //Right front
#define M5_SW               PN_5 //Right middle
#define M6_SW               PN_4 //Right back

const int switches[6] = {M1_SW, M2_SW, M3_SW, M4_SW, M5_SW, M6_SW};

//////////////Outputs//////////////////////
#define RSPEED              PD_4
#define LSPEED              PD_5
#define WATCHDOG            PB_5

//////////////////////////////////////////////////
// We send serial speed bytes to motor controllers 
const byte DRIVE_MAX_FORWARD   = 255;
const byte DRIVE_MAX_REVERSE   = 0;
const byte DRIVE_ZERO          = 127;
int8_t left_drive_speed        = DRIVE_ZERO;
int8_t right_drive_speed       = DRIVE_ZERO;

bool notification_on           = 0;

int8_t front_motor;  //serial 3
int8_t middle_motor; //serial 4
int8_t back_motor;   //serial 6
//need left and right actual speed

// need variales for each motor
/*

//RoveWatchdog        Watchdog;

void roveEstopDriveMotors();

/////////////////////////////////////////////////////////////////////////////////////////////////
*/
void setup()
{
  Serial.begin( 9600); 
  Serial3.begin(19200);
  Serial4.begin(19200);
  Serial6.begin(19200);
  
  delay(1);
  
//need begins
  

  
  //Watchdog.begin(roveEstopDriveMotors, 150); 
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop()
{
 








  (M1_SW && M2_SW && M3_SW)? digitalWrite(LSPEED, HIGH):digitalWrite(LSPEED, LOW); //If left wheels moving then led comes on
  (M4_SW && M5_SW && M6_SW)? digitalWrite(RSPEED, HIGH):digitalWrite(RSPEED, LOW); //same as above but for the right
}
/////////////////////////////^^^^^NEW^^^^^///////////////////////////
 /* 
  uint16_t data_id   = 0;
  size_t   data_size = 0;
  uint8_t  data_value[4];
  roveComm_GetMsg(&data_id, &data_size, &data_value);
  switch (data_id) 
  {     
    case DRIVE_LEFT_RIGHT:
    {       
      int32_t speed            = *(int32_t*)(data_value);     
      int16_t left_speed_temp  =  (int16_t) (speed >> 16); // 2 high bytes contain RED's left speed  as int16
      int16_t right_speed_temp =  (int16_t)  speed;        // 2 low  bytes contain RED's right speed as int16
    
     // left_speed_temp   = -left_speed_temp; // Motors were wired backwards     
      left_drive_speed  = map(left_speed_temp,  RED_MAX_REVERSE, RED_MAX_FORWARD, DRIVE_MAX_REVERSE, DRIVE_MAX_FORWARD); 
      right_drive_speed = map(right_speed_temp, RED_MAX_REVERSE, RED_MAX_FORWARD, DRIVE_MAX_REVERSE, DRIVE_MAX_FORWARD);     
      Watchdog.clear();
      break;  
    }

  Serial2.write(left_drive_speed );
  Serial3.write(left_drive_speed );
  Serial4.write(left_drive_speed );  
  Serial5.write(right_drive_speed);
  Serial6.write(right_drive_speed);
  Serial7.write(right_drive_speed);
  delay(1);

*/

////////////////////////////////

void roveEstopDriveMotors() 
{ 
  left_drive_speed  = DRIVE_ZERO;
  right_drive_speed = DRIVE_ZERO;
  //Watchdog.clear();
}
