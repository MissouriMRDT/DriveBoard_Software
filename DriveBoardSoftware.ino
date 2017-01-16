// include standard C integers
#include <stdint.h>

// Include Tiva Watchdog Driver Library
#include <driverlib/sysctl.h>
#include <driverlib/watchdog.h>
#include <driverlib/rom_map.h>

// Libraries used by RoveWare
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

// Roveware libraries
#include "RoveEthernet.h"
#include "RoveComm.h"

const byte HOLD_TIME = 50; // time to hold the speed in ms. (for test routine only)
// constants for RoveComm
const uint16_t SYSTEM_ABORT  = 199;
const uint16_t LEFT_SIDE_IP  = 100;
const uint16_t RIGHT_SIDE_IP = 101;
const uint32_t ROVECOMM_TIMEOUT = 16400000; // ~1.5 seconds
const uint32_t ROVECOMM_WATCHDOG = WATCHDOG1_BASE;

// speed constants
const byte MAX_FORWARD = 255;
const byte ZERO_SPEED = 127;
const byte MAX_REVERSE = 0;

// speed commands sent from RED XBox Controller values
const short RED_MAX = 1000;
const short RED_MIN = -1000;

byte speed = 100; // speed variable for test routine
byte speeds[6];   // array of speeds for each motor

short last_time = 0;

void setup()
{
  // begin RoveComm with assigned IP
  roveComm_begin(192,168,1,130); 
  // open motor controller serial channels
  Serial1.begin(19200);
  Serial2.begin(19200);
  Serial3.begin(19200);
  Serial4.begin(19200);
  Serial5.begin(19200);
  Serial6.begin(19200);
  Serial7.begin(19200);

  // test routine for motor controller communication (remove for real operation)
  test();
  
  // set speeds to zero to begin the loop
  speed = ZERO_SPEED;
  for(byte i = 0; i < 6; i++)
  {
    speeds[i] = ZERO_SPEED;
  }
  RoveCommWatchdog_begin(roveEstopDriveMotors, RC_PWM_ZERO_SPEED); //! cant find the definition for this function
}
void loop() 
{  
  
  Serial1.write(speed);
  Serial2.write(speed);
  Serial3.write(speed);
  Serial4.write(speed);
  Serial5.write(speed);
  Serial6.write(speed);
  Serial7.write(speed);  
}

void roveWatchdogClear()
{
  WatchdogIntClear(ROVECOMM_WATCHDOG);
}
void Estop()
{
  for(byte i = 0; i < 6; i++)
  {
    speeds[i] = 0;
  }
  write_speeds();
}
void write_speeds()
{
  // need to determine the channels correlations to the motors
  Serial1.write(speeds[0]);
  Serial2.write(speeds[0]);
  Serial3.write(speeds[0]);
  Serial4.write(speeds[0]);
  Serial5.write(speeds[0]);
  Serial6.write(speeds[0]);
  Serial7.write(speeds[0]);
  delay(1); //! determine if a delay is necessary after writing the speeds
}
void forward_cycle()
{
  short accel = 1;
  speed = ZERO_SPEED+1; 
  last_time = 0;
  
  while(speed >= ZERO_SPEED)
  {
    if(millis() - last_time >= HOLD_TIME) // keep sending speeds for HOLD_TIME milliseconds
    {
      speed += accel; // add acceleration to the speed.

      if(speed >= MAX_FORWARD) // if speed hits max...
        accel *= -1; // negate acceleration.
        
      last_time = millis(); // update time
      Serial.print(speed + " "); // output the current speed      
    }
    // write speeds
    Serial1.write(speed);
    Serial2.write(speed);
    Serial3.write(speed);
    Serial4.write(speed);
    Serial5.write(speed);
    Serial6.write(speed);
    Serial7.write(speed);
  }
  Serial.println();
}

void reverse_cycle()
{
  short accel = 1;
  speed = ZERO_SPEED - 1;
  last_time = 0;
  
  while(speed <= ZERO_SPEED)
  {
    if(millis() - last_time >= HOLD_TIME) // keep sending speed fot HOLD_TIME milliseconds
    {
      speed -= accel; // add acceleration to speed (negative accel increases speed since it is increasing in reverse direction.

      if(speed <= MAX_REVERSE) // if speed hits max...
        accel *= -1; // negate acceleration.

      last_time = millis(); // update time
      Serial.print(speed + " "); // output the current speed          
    }
    // write speeds
    Serial1.write(speed);
    Serial2.write(speed);
    Serial3.write(speed);
    Serial4.write(speed);
    Serial5.write(speed);
    Serial6.write(speed);
    Serial7.write(speed);
  }
  Serial.println();
}
void test()
{
  delay(2000); // wait for a bit before starting test routine
  forward_cycle();
  reverse_cycle();
}





