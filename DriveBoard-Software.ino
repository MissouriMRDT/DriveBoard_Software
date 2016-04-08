// Standard C integers of specific size (such as int16_t)
#include <stdint.h>

// Include Tiva Watchdog Driver Library
#include <driverlib/sysctl.h>
#include <driverlib/watchdog.h>
#include <driverlib/rom_map.h>

#include <stdio.h>
#include <stdlib.h>

// Energia libraries used by RoveWare itself
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

// RoveWare 
#include "RoveEthernet.h"
#include "RoveComm.h"

// DRIVE DATE : CONTINOUS ROTATION SERVO BY PWM
//
// https://en.wikipedia.org/wiki/Servo_(radio_control)

// Arduino Library for Rc Pwm Servo (rotate joint angle): min 0 maps to 544 microseconds, max 360 maps to 2400 microseconds
#include <Servo.h>

///////////////////////////////DriveBoard-Software Loop under developement

// RED sends us a data_id 100 or 101 with data_size of two bytes and data is a two byte motor_speed
uint16_t data_id = 0;
size_t data_size = 0; 

const uint16_t DRIVE_DEVICE_SYSTEM_ABORT = 199;
bool system_abort_flag = false;

const uint16_t DRIVE_LEFT_MOTORS_BY_IP = 100;
const uint16_t DRIVE_RIGHT_MOTORS_BY_IP = 101;

// Rc Pwm protocol for (spin wheel speed) bis not by position : full reverse maps to 1000 microseconds, full forward maps to 2000 microseconds
const int RC_PWM_MAX_SPEED      =   2000;
const int RC_PWM_ZERO_SPEED     =   1500;
const int RC_PWM_MIN_SPEED      =   1000;

// Red sends xbox controller input to us
const int16_t RED_MAX_SPEED     =   1000;
const int16_t RED_MIN_SPEED     =  -1000;

int16_t motors_speed = 0;

// Servo goes a position between 0 and 360
int position = 0;

Servo motor_left_front;
Servo motor_left_middle;
Servo motor_left_back;

Servo motor_right_front;
Servo motor_right_middle;
Servo motor_right_back;

///////////////////////////////Utilities under developement
//RoveCommWatchdog.h

//for 120 Mhz : 8.333 nanoSec period
//TODO const uint32_t ROVECOMM_WATCHDOG_TIMEOUT_TICKS = F_CPU*ROVECOMM_WATCHDOG_TIMEOUT_SECONDS;
//todo map(time_out_microseconds, ROVECOMM_WATCHDOG_TIMEOUT_TICKS);
//Randomly chosen number, takes roughly 1.5 seconds
const uint32_t ROVECOMM_WATCHDOG_TIMEOUT_TICKS = 30000000; 
const uint32_t ROVECOMM_WATCHDOG = WATCHDOG1_BASE;

void roveWatchdogClear()
{
  WatchdogIntClear(ROVECOMM_WATCHDOG);
}//end fnctn

void RoveCommWatchdog_begin(void (*roveWatchdogIntHandler)(void), const uint32_t time_out_millis)
{   
  // Enable Watchdog Timer 1; supposedly timer 0 has a conflict in Energia, unconfirmed
  SysCtlPeripheralEnable(SYSCTL_PERIPH_WDOG1);  
  WatchdogUnlock(ROVECOMM_WATCHDOG);
  WatchdogReloadSet(ROVECOMM_WATCHDOG, ROVECOMM_WATCHDOG_TIMEOUT_TICKS);
  
  WatchdogIntRegister(ROVECOMM_WATCHDOG, roveWatchdogIntHandler);
  
  WatchdogIntEnable(ROVECOMM_WATCHDOG);
  WatchdogResetDisable(ROVECOMM_WATCHDOG);
  WatchdogEnable(ROVECOMM_WATCHDOG);

}//end fnctn
///////////////////////////////



void setup()
{
  //Serial.begin(9600);  
  roveComm_Begin(192, 168, 1, 130); 
  
  motor_left_front.attach(37, RC_PWM_MIN_SPEED, RC_PWM_MAX_SPEED);    //Tiva Pin: PG0 = 37
  motor_left_middle.attach(38, RC_PWM_MIN_SPEED, RC_PWM_MAX_SPEED);   //Tiva Pin: PF3 = 38
  motor_left_back.attach(39, RC_PWM_MIN_SPEED, RC_PWM_MAX_SPEED);     //PF2
  
  motor_right_front.attach(80, RC_PWM_MIN_SPEED, RC_PWM_MAX_SPEED);   //PG1 
  motor_right_middle.attach(79, RC_PWM_MIN_SPEED, RC_PWM_MAX_SPEED);  //PK4
  motor_right_back.attach(78, RC_PWM_MIN_SPEED, RC_PWM_MAX_SPEED);    //PK_5
  
  RoveCommWatchdog_begin(roveEstopDriveMotors, RC_PWM_ZERO_SPEED);
}//end setup

void loop()
{
  //Provide System Abort
  if(!system_abort_flag)
  { 
    //If there is no message data_id gets set to zero
    roveComm_GetMsg(&data_id, &data_size, &motors_speed);
    
    switch (data_id) 
    {   
      //Don't do anything for data_id zero 
      case 0:
        break;
        
      case DRIVE_LEFT_MOTORS_BY_IP: 
      
        //This is the tricky part. We are hacking a 0 to 360 angle control servo library to write out continous rotation speed by mapping one to the other
        position = map((int)motors_speed, RED_MIN_SPEED, RED_MAX_SPEED, 0,  180); 
      
        motor_left_front.write(position);
        motor_left_middle.write(position);
        motor_left_back.write(position);
        //Serial.print("left motors position :); 
        //Serial.println(position);
        //Serial.print("left motors speed :); 
        //Serial.println(motors_speed);
        //delay(100);
        break;
      
      case DRIVE_RIGHT_MOTORS_BY_IP: 
      
        position = map((int)motors_speed, RED_MIN_SPEED, RED_MAX_SPEED, 0,  180); 
      
        motor_right_front.write(position);
        motor_right_middle.write(position);  
        motor_right_back.write(position);
        //Serial.print("right motors position :); 
        //Serial.println(position);  
        //Serial.print("right motors speed :); 
        //Serial.println(motors_speed);
        //delay(100);
        break;
      
      default:
        //Serial.print("RoveDrive unrecognized data_id :");
        //Serial.println(data_id);
        //delay(100);
        break;  
    }//endswitch
    
    if (data_id != 0) 
    {
      roveWatchdogClear();
    }//end if
    
  }else{   
    roveComm_SendMsg(DRIVE_DEVICE_SYSTEM_ABORT, data_size, &system_abort_flag);
    //Serial.print("system_abort_flag! : "); 
    //Serial.println(system_abort_flag); 
    delay(1000); 
  }//end if
}//end loop



void roveEstopDriveMotors() 
{ 
  motor_left_front.write(0);
  motor_left_middle.write(0);
  motor_left_back.write(0);
  
  motor_right_front.write(0);
  motor_right_middle.write(0);  
  motor_right_back.write(0);
  //Serial.println("Watchdog Triggered");
  
  roveWatchdogClear();
}//end fnctn
