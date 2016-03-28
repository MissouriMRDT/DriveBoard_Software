// Standard C integers of specific size (such as int16_t)
#include <stdint.h>

// Include Tiva Watchdog Driver Library
#include <driverlib/sysctl.h>
#include <driverlib/watchdog.h>

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

// Servo goes a position between 0 and 360
int position = 0;

Servo left_front_motor;
Servo left_middle_motor;
Servo left_back_motor;

Servo right_front_motor;
Servo right_middle_motor;
Servo right_back_motor;

// Rc Pwm protocol for (spin wheel speed) bis not by position : full reverse maps to 1000 microseconds, full forward maps to 2000 microseconds 
const int RC_PWM_MIN_SPEED      =   1000;
const int RC_PWM_MAX_SPEED      =   2000;

// RED sends us a data_id and a motor_speed
uint16_t data_id = 0;

const uint16_t DRIVE_LEFT_MOTORS_BY_IP = 100;
const uint16_t DRIVE_RIGHT_MOTORS_BY_IP = 101;

// Not used: We already know the size of motor_speed (int16_t), but Rovecomm does supports variable command sizes, so we always pass in data_size
size_t data_size = 0; 

// Red sends xbox controller input to us
const int16_t RED_MIN_SPEED     =  -1000;
const int16_t RED_MAX_SPEED     =   1000;

const int position_zero = map(0, RED_MIN_SPEED, RED_MAX_SPEED, 0,  180);

int16_t motors_speed = 0;

uint32_t ROVECOMMWATCHDOG = WATCHDOG1_BASE;
#define WATCHDOG_LENGTH 30000000 //Randomly chosen number, takes roughly 1.5 seconds

void stopDriving() { 
  //Serial.println("Watchdog Triggered");
  
  left_front_motor.write(position_zero);
  left_middle_motor.write(position_zero);
  left_back_motor.write(position_zero);
  right_front_motor.write(position_zero);
  right_middle_motor.write(position_zero);  
  right_back_motor.write(position_zero);
  
  WatchdogIntClear(ROVECOMMWATCHDOG);
}//end function

void setup()
{
  //Serial.begin(9600);  
  roveComm_Begin(192, 168, 1, 130); 
  
  //Tiva Pin: PG0 = 37
  left_front_motor.attach(37, RC_PWM_MIN_SPEED, RC_PWM_MAX_SPEED);
  //Tiva Pin: PF3 = 38
  left_middle_motor.attach(38, RC_PWM_MIN_SPEED, RC_PWM_MAX_SPEED);
  //PF2
  left_back_motor.attach(39, RC_PWM_MIN_SPEED, RC_PWM_MAX_SPEED);
  
  //PG1
  right_front_motor.attach(80, RC_PWM_MIN_SPEED, RC_PWM_MAX_SPEED);  
  //PK4
  right_middle_motor.attach(79, RC_PWM_MIN_SPEED, RC_PWM_MAX_SPEED);
  //PK_5
  right_back_motor.attach(78, RC_PWM_MIN_SPEED, RC_PWM_MAX_SPEED);
  
  // Enable Watchdog Timer 1; supposedly timer 0 has a conflict in Energia, unconfirmed
  SysCtlPeripheralEnable(SYSCTL_PERIPH_WDOG1);
  
  WatchdogUnlock(ROVECOMMWATCHDOG);
  WatchdogReloadSet(ROVECOMMWATCHDOG, WATCHDOG_LENGTH);
  WatchdogIntRegister(ROVECOMMWATCHDOG, stopDriving);
  WatchdogIntEnable(ROVECOMMWATCHDOG);
  WatchdogResetDisable(ROVECOMMWATCHDOG);
  WatchdogEnable(ROVECOMMWATCHDOG);
  
}//end setup

void loop()
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
      
      left_front_motor.write(position);
      left_middle_motor.write(position);
      left_back_motor.write(position); 
      //Serial.println(motors_speed);
      //Serial.println(pos);
      break;
    
    case DRIVE_RIGHT_MOTORS_BY_IP: 
    
      position = map((int)motors_speed, RED_MIN_SPEED, RED_MAX_SPEED, 0,  180); 
      
      right_front_motor.write(position);
      right_middle_motor.write(position);  
      right_back_motor.write(position); 
      //Serial.println(motors_speed);
      //Serial.println(pos); 
      break;
    
    default:
      //Serial.print("RoveDrive unrecognized data_id :");
      //Serial.println(data_id);
      break;
  
  }//endswitch
  
  if (data_id != 0) {
    WatchdogIntClear(ROVECOMMWATCHDOG);
  }//end if

}//end loop
