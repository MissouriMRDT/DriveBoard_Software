/*
 * DriveBoard Software Rev 1 2019
 * Used with DriveBoard Rev 1 2019
 *
 * Brian Dahlman
 * Takes drive commands from RoveComm and sends the remapped instructions to the motor controllers
 */
#include <Energia.h>
#include "RoveComm.h"

RoveCommEthernetUdp RoveComm;

//////////////Inputs///////////////////////
#define DIRECTION_SW           PD_2
#define M1_SW                  PK_2 //Left front     serial3
#define M2_SW                  PK_3 //Right front    serial3
#define M3_SW                  PQ_0 //Left Mid      serial4
#define M4_SW                  PP_4 //Right Mid    serial4
#define M5_SW                  PN_5 //Left rear   serial6
#define M6_SW                  PN_4 //Right rear     serial6

const int switches[6] = {M1_SW, M2_SW, M3_SW, M4_SW, M5_SW, M6_SW};

#define TX_1                       PK_1
#define TX_2                       PP_1
#define TX_3                       PA_5

#define RSPEED_IND                 PD_4
#define LSPEED_IND                 PD_5
#define WATCHDOG_IND               PB_5

//////////////////////////////////////////////////
// We send serial speed bytes to motor controllers 
#define LED_MAX_INTENSITY          255
#define LED_MIN_INTENSITY          0
#define DRIVE_MAX_FORWARD          64
#define DRIVE_ZERO                 0
//#define DRIVE_ZERO               127
#define RED_MAX_FORWARD            1000
#define RED_ZERO                   0

#define SWITCHMOTORSPEED           100
#define RC_DELAY                   10

int16_t motorSpeeds[6]             = {0,0,0,0,0,0};
int8_t direction;

int8_t LEFT_SPEED_IND;
int8_t RIGHT_SPEED_IND;
//int8_t front_motors;  //serial 3
//int8_t middle_motors; //serial 4
//int8_t back_motors;   //serial 6

//RoveWatchdog        Watchdog;

void setup()
{
  Serial.begin(9600); 
  Serial1.begin(9600);
  Serial2.begin(9600);
  Serial3.begin(9600); //motors 1 & 2   front
  Serial4.begin(9600); //motors 3 & 4   middle
  Serial5.begin(9600);
  Serial6.begin(9600); //motors 5 & 6   back
  Serial7.begin(9600);
  pinMode(LSPEED_IND, OUTPUT);
  pinMode(RSPEED_IND, OUTPUT);
  pinMode(TX_1, OUTPUT);
  pinMode(TX_2, OUTPUT);
  pinMode(TX_3, OUTPUT);
  RoveComm.begin(RC_DRIVEBOARD_FOURTHOCTET);
  delay(RC_DELAY);
//Watchdog.begin(roveEstopDriveMotors, 150); 
}


void loop()
{
  delay(100);
 // digitalWrite(RSPEED_IND, HIGH);
 // digitalWrite(LSPEED_IND, HIGH);
  rovecomm_packet packet;
  packet = RoveComm.read();
  if(packet.data_id!=0)
  {
    Serial.println(packet.data_id);
    Serial.println(packet.data_count);
    for(int i = 0; i<packet.data_count; i++)
    {
      Serial.println(packet.data[i]);
    }
    switch(packet.data_id)
    {
      case RC_DRIVEBOARD_DRIVELEFTRIGHT_DATAID:
      {
        //int speed[] = {LSPEED, RSPEED}; //speed values range from -1k to 1k
        //speed[0] = packet.data[0];
        if(packet.data[0] < -1000)
          packet.data[0] = -1000;
        if(packet.data[0] > 1000)
          packet.data[0] = 1000;
        if(packet.data[1] > 1000)
          packet.data[1] = 1000;
        if(packet.data[1] < -1000)
          packet.data[1] = -1000;
        //speed[1] = packet.data[1];
        motorSpeeds[0] = packet.data[0];//data0
        motorSpeeds[1] = packet.data[1];//data0
        motorSpeeds[2] = packet.data[0];//data0
        motorSpeeds[3] = packet.data[1];//data1
        motorSpeeds[4] = packet.data[0];//data1
        motorSpeeds[5] = packet.data[1];//data1
        break;
      }
      case RC_DRIVEBOARD_SPEEDRAMPVALUEs_DATAID:
      {
        break;
      }
      default:
        break;
    }
   }
     for(int i = 0; i <6;i++)
    {
      Serial.println(motorSpeeds[i]);
    }
////////////////Motor control///////////////////
  
  motorSpeedOverride();
  setMotorControllerSpeeds();
////////////////////LEDs////////////////////////
  analogWrite(LSPEED_IND, map(abs(motorSpeeds[0]), RED_ZERO, RED_MAX_FORWARD, LED_MIN_INTENSITY, LED_MAX_INTENSITY)); //remaps motor speed values from 0-1000 to 0-255
  analogWrite(RSPEED_IND, map(abs(motorSpeeds[1]), RED_ZERO, RED_MAX_FORWARD, LED_MIN_INTENSITY, LED_MAX_INTENSITY));//then sets the analog value to the corresponding value
} 

///////////////FUNCTIONS//////////////////
void motorSpeedOverride()
{
  
  direction = (digitalRead(DIRECTION_SW) == HIGH)? 1:-1;
  for(int i = 0; i<6; i++)
  {
    motorSpeeds[i] = (digitalRead(switches[i]) == HIGH)? (SWITCHMOTORSPEED*direction): motorSpeeds[i];
  }
  return;
}

void setMotorControllerSpeeds()//sends drive speed
{
  for(int i = 0; i < 6 ; i++)
  {
    byte speed                  = B00000000;
    byte direction              = B00000000;
    byte motor                  = B00000000;
    int8_t motor_speeds_sent[6] = {0,0,0,0,0,0};
    int8_t curr_motor = i;
    
    (motorSpeeds[curr_motor] > 0) ? (direction = B01000000):(direction = B00000000); //if motorspeed is positive then forward else reverse
    
    if(curr_motor == 0)//motor 1 (front left)
    {  
      motor_speeds_sent[0] = map(abs(motorSpeeds[curr_motor]), RED_ZERO, RED_MAX_FORWARD, DRIVE_ZERO, DRIVE_MAX_FORWARD);//remaps motorspeeds to 0-64 for transmitting via serial
      motor = B00000000;
      Serial1.write(speed | direction | motor);
Serial.print("motor 1: ");
Serial.println(motor_speeds_sent[0] | direction | motor, BIN);
    }
    if(curr_motor == 1)//motor 2 (front right)
    { 
      motor_speeds_sent[1] = map(abs(motorSpeeds[curr_motor]), RED_ZERO, RED_MAX_FORWARD, DRIVE_ZERO, DRIVE_MAX_FORWARD);
      motor = B10000000;
      Serial2.write(speed | direction | motor);
Serial.print("motor 2: ");
Serial.println(motor_speeds_sent[1] | direction | motor, BIN);
    }
    if(curr_motor == 2)//motor 3 (middle left)
    {  
      motor_speeds_sent[2] = map(abs(motorSpeeds[curr_motor]), RED_ZERO, RED_MAX_FORWARD, DRIVE_ZERO, DRIVE_MAX_FORWARD);  
      motor = B00000000;
      Serial3.write(speed | direction | motor);
Serial.print("motor 3: ");
Serial.println(motor_speeds_sent[2] | direction | motor, BIN);
    }
    if(curr_motor == 3)//motor 4 (middle right)
    {    
      motor_speeds_sent[3] = map(abs(motorSpeeds[curr_motor]), RED_ZERO, RED_MAX_FORWARD, DRIVE_ZERO, DRIVE_MAX_FORWARD); 
      motor = B10000000;
      Serial4.write(speed | direction | motor);
Serial.print("motor 4: ");
Serial.println(motor_speeds_sent[3] | direction | motor, BIN);
    }
    if(curr_motor == 4)//motor 5 (rear left)
    {
      motor_speeds_sent[4] = map(abs(motorSpeeds[curr_motor]), RED_ZERO, RED_MAX_FORWARD, DRIVE_ZERO, DRIVE_MAX_FORWARD);
      motor = B00000000;
      Serial5.write(speed | direction | motor);
Serial.print("motor 5: ");
Serial.println(motor_speeds_sent[4] | direction | motor, BIN);
    }
    if(curr_motor == 5)//motor 6 (rear right)
    {
      motor_speeds_sent[5] = map(abs(motorSpeeds[curr_motor]), RED_ZERO, RED_MAX_FORWARD, DRIVE_ZERO, DRIVE_MAX_FORWARD);
      motor = B10000000;
      Serial6.write(speed | direction | motor);
Serial.print("motor 6: ");
Serial.println(motor_speeds_sent[5] | direction | motor, BIN);
    }  
  }
  return;
}
