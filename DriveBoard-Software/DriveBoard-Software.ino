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
#include <Servo.h>

// RoveWare 
#include "RoveEthernet.h"
#include "RoveComm.h"

// RED sends us a data_id with data_size of two bytes and data is a two byte motor_speed
uint16_t data_id = 0;
size_t data_size = 0;
uint8_t data_value[4];

// RoveComm Codes
const uint16_t DRIVE_DATA_ID = 528;
const uint16_t LED_RGB_CTRL = 2320;
const uint16_t LED_FUNCTION = 2321;
const uint16_t HEADLIGHT_CTRL = 2336;
const uint16_t DROP_BAY_OPEN = 1584;
const uint16_t DROP_BAY_CLOSE = 1585;

//Pin Assignments
#define SERVO_0_PIN PM_3
#define SERVO_1_PIN	PN_2
#define SERVO_2_PIN	PM_7
#define LED_STRIP_PIN PP_5 //SERVO 3
#define HEADLIGHT_PIN PM_6


// Speed consts
const byte MAX_FORWARD = 255;
const byte ZERO_SPEED  = 127;
const byte MAX_REVERSE = 0;

// RED sends xbox controller input to us
const int16_t RED_MAX_SPEED     =   1000;
const int16_t RED_MIN_SPEED     =  -1000;

byte right_speed = 0;
byte left_speed = 0;

Servo Dropbay[2];

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
  delay(1);

  pinMode(LED_STRIP_PIN, OUTPUT);
  pinMode(HEADLIGHT_PIN, OUTPUT);

  digitalWrite(HEADLIGHT_PIN, 0);

  // pair servos to correct PWM pins
  Dropbay[0].attach(SERVO_0_PIN);
  Dropbay[1].attach(SERVO_1_PIN);

  Dropbay[0].write(0);
  Dropbay[1].write(0);

  right_speed = ZERO_SPEED;
  left_speed = ZERO_SPEED;
  
  RoveCommWatchdog_begin(roveEstopDriveMotors, 1500); // not sure what second arg is... used to be RC_PWM_ZERO_SPEED
}

void loop()
{
    //If there is no message data_id gets set to zero
    roveComm_GetMsg(&data_id, &data_size, &data_value);
    if (data_id)
    {
      Serial.print("data_id: ");
      Serial.println(data_id);
      Serial.print("data_value: ");
      Serial.println(data_value[0]);
    }

    switch (data_id) 
    {   
      
      //Don't do anything for data_id zero 
      case 0:
        break;

      case DRIVE_DATA_ID:
      {
      	int32_t speed = *(int32_t*)(data_value);
        int16_t left_temp, right_temp;
        
        left_temp = (int16_t)(speed >> 16);
        right_temp = (int16_t)speed;
		
        left_speed = map(left_temp, 1000, -1000, 0, 255); //change the signs on the "1000"s to change the direction wheels spin when moving forward with the xbox controller
        right_speed = map(right_temp, -1000, 1000, 0, 255);
      }
        break;

      case DROP_BAY_OPEN:
        if (data_value[0] == 0)
        {
          Dropbay[0].write(255);
          Serial.println("Drop Bay 0 Open");
        }
        else if (data_value[0] == 1)
        {
          Dropbay[1].write(255);
          Serial.println("Drop Bay 1 Open");
        }
        break;

      case DROP_BAY_CLOSE:
        if (data_value[0] == 0)
        {
          Dropbay[0].write(0);
          Serial.println("Drop Bay 0 Closed");
        }
        else if (data_value[0] == 1)
        {
          Dropbay[1].write(0);
          Serial.println("Drop Bay 1 Closed");
        }
        break;

      case HEADLIGHT_CTRL:
      	digitalWrite(HEADLIGHT_PIN, data_value[0]);
      	break;
        
      default:
        break;  
    }
    if (data_id != 0)
    {
      roveWatchdogClear();
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

// Rotates individual motors at MAX_FORWARD data_value for 1 second each

