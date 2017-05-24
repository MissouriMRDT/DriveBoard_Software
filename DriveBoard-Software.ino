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

// RED sends us a data_id 100 or 101 with data_size of two bytes and data is a two byte motor_speed
uint16_t data_id = 0;
size_t data_size = 0; 
bool system_abort_flag = false;

// RoveComm Codes
const uint16_t DRIVE_LEFT_MOTORS_BY_IP = 100;
const uint16_t DRIVE_RIGHT_MOTORS_BY_IP = 101;
const uint16_t DRIVE_DEVICE_SYSTEM_ABORT = 199;
const uint16_t DRIVE_DATA_ID = 528;

// Speed consts
const byte MAX_FORWARD = 255;
const byte ZERO_SPEED  = 127;
const byte MAX_REVERSE = 0;

// RED sends xbox controller input to us
const int16_t RED_MAX_SPEED     =   1000;
const int16_t RED_MIN_SPEED     =  -1000;

int32_t speed = 0;
byte right_speed = 0;
byte left_speed = 0;

const uint32_t ROVECOMM_WATCHDOG_TIMEOUT_TICKS = 5500000; // ~0.5 seconds
const uint32_t ROVECOMM_WATCHDOG = WATCHDOG1_BASE;

void roveWatchdogClear()
{
  WatchdogIntClear(ROVECOMM_WATCHDOG);
}

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
}

void setup()
{
  Serial.begin(9600);  
  roveComm_Begin(192, 168, 1, 130); 
  // Open RS232 Channels to the motor controllers
  Serial2.begin(19200);
  Serial3.begin(19200);
  Serial4.begin(19200);
  Serial5.begin(19200);
  Serial6.begin(19200);
  Serial7.begin(19200);

  pinMode(31, OUTPUT);
  
  right_speed = ZERO_SPEED;
  left_speed = ZERO_SPEED;
  
  RoveCommWatchdog_begin(roveEstopDriveMotors, 1500); // not sure what second arg is... used to be RC_PWM_ZERO_SPEED
}

void loop()
{
  //Provide System Abort
  if(!system_abort_flag)
  { 
    //If there is no message data_id gets set to zero
    roveComm_GetMsg(&data_id, &data_size, &speed);


    switch (data_id) 
    {   
      //Don't do anything for data_id zero 
      case 0:
        break;
        
      case DRIVE_LEFT_MOTORS_BY_IP: 
        speed = map(speed, -1000, 1000, 0, 255);
        left_speed = speed;
        break;
      
      case DRIVE_RIGHT_MOTORS_BY_IP: 
        speed = map(speed, 1000, -1000, 0, 255);
        right_speed = speed;
        break;

      case DRIVE_DATA_ID:
        int16_t left_temp, right_temp;
        
        left_temp = (int16_t)(speed >> 16);
        right_temp = (int16_t)speed;
		
        left_speed = map(left_temp, -1000, 1000, 0, 255);
        right_speed = map(right_temp, 1000, -1000, 0, 255);
        break;
        
      default:
        break;  
    }
    if (data_id != 0)
    {
      roveWatchdogClear();
    }
  }
  else
  {   
    roveComm_SendMsg(DRIVE_DEVICE_SYSTEM_ABORT, sizeof(system_abort_flag), &system_abort_flag);
    delay(1000); 
  }
  write_speeds();
}

void roveEstopDriveMotors() 
{ 
  right_speed = ZERO_SPEED;
  left_speed  = ZERO_SPEED;
  Serial.println("Watchdog Triggered");
  roveWatchdogClear();
}

void write_speeds()
{
  Serial2.write(left_speed);
  Serial3.write(left_speed);
  Serial4.write(left_speed);  
  
  Serial5.write(right_speed);
  Serial6.write(right_speed);
  Serial7.write(right_speed);

  delay(1);
}

// Rotates individual motors at MAX_FORWARD speed for 1 second each
// Used to determine which Serial channel corresponds to each wheel
void test_individual()
{
  int t;
  t = millis();
  while(millis() < t+1000)
  {
    Serial2.write(MAX_FORWARD);
    Serial3.write(ZERO_SPEED);
    Serial4.write(ZERO_SPEED);
    Serial5.write(ZERO_SPEED);
    Serial6.write(ZERO_SPEED);
    Serial7.write(ZERO_SPEED);
  }
  t = millis();
  while(millis() < t+1000)
  {
    Serial2.write(ZERO_SPEED);
    Serial3.write(MAX_FORWARD);
    Serial4.write(ZERO_SPEED);
    Serial5.write(ZERO_SPEED);
    Serial6.write(ZERO_SPEED);
    Serial7.write(ZERO_SPEED);
  }
  t = millis();
  while(millis() < t+1000)
  {
    Serial2.write(ZERO_SPEED);
    Serial3.write(ZERO_SPEED);
    Serial4.write(MAX_FORWARD);
    Serial5.write(ZERO_SPEED);
    Serial6.write(ZERO_SPEED);
    Serial7.write(ZERO_SPEED);
  }
  t = millis();
  while(millis() < t+1000)
  {
    Serial2.write(ZERO_SPEED);
    Serial3.write(ZERO_SPEED);
    Serial4.write(ZERO_SPEED);
    Serial5.write(MAX_FORWARD);
    Serial6.write(ZERO_SPEED);
    Serial7.write(ZERO_SPEED);
  }
  t = millis();
  while(millis() < t+1000)
  {
    Serial2.write(ZERO_SPEED);
    Serial3.write(ZERO_SPEED);
    Serial4.write(ZERO_SPEED);
    Serial5.write(ZERO_SPEED);
    Serial6.write(MAX_FORWARD);
    Serial7.write(ZERO_SPEED);
  }
  t = millis();
  while(millis() < t+1000)
  {
    Serial2.write(ZERO_SPEED);
    Serial3.write(ZERO_SPEED);
    Serial4.write(ZERO_SPEED);
    Serial5.write(ZERO_SPEED);
    Serial6.write(ZERO_SPEED);
    Serial7.write(MAX_FORWARD);
  }
  delay(3000);
}

