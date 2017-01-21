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

// Speed consts
const byte MAX_FORWARD = 255;
const byte ZERO_SPEED  = 127;
const byte MAX_REVERSE = 0;

// RED sends xbox controller input to us
const int16_t RED_MAX_SPEED     =   1000;
const int16_t RED_MIN_SPEED     =  -1000;

int16_t speed = 0;
byte right_speed = 0;
byte left_speed = 0;

const uint32_t ROVECOMM_WATCHDOG_TIMEOUT_TICKS = 16400000; // ~1.5 seconds
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
  // Serial Assignments:
  //  Serial2: Rear Left
  //  Serial3: Front Left
  //  Serial4: Middle Left
  //  Serial5: Middle Right
  //  Serial6: Front Right
  //  Serial7: Rear Right
  Serial2.begin(19200);
  Serial3.begin(19200);
  Serial4.begin(19200);
  Serial5.begin(19200);
  Serial6.begin(19200);
  Serial7.begin(19200);

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

    speed += 1000; // change possible range to [0, 1999]
    speed = (speed*MAX_FORWARD)/1999; // adjust the value for expected speed range
    switch (data_id) 
    {   
      //Don't do anything for data_id zero 
      case 0:
        break;
        
      case DRIVE_LEFT_MOTORS_BY_IP: 
        left_speed = speed;
        Serial.println(left_speed);
        break;
      
      case DRIVE_RIGHT_MOTORS_BY_IP: 
        right_speed = speed;
        Serial.println(right_speed);
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
    roveComm_SendMsg(DRIVE_DEVICE_SYSTEM_ABORT, data_size, &system_abort_flag);
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
  delay(1); //! determine if delay is necessary
}

