//DriveBoard Software Rev 1 2021

#include "DriveBoardSoftware.h"
#include "driverlib/can.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"

/////////////////////////////////////////////////////////////////////////
EthernetServer TCPServer(RC_ROVECOMM_DRIVEBOARD_PORT);

////////////////////////////////////////////////////////////////
void setup() {
  // initialize all serial ports: 
  Serial.begin(19200);
  // Motor speed serial ports
  FL_SERIAL.begin(19200);                  //FL
  FR_SERIAL.begin(19200);                  //FR
  RL_SERIAL.begin(19200);                  //RL
  RR_SERIAL.begin(19200);                  //RR
  // ODrive Serial Ports
  LeftOdrive.begin(LEFT_ODRIVE_SERIAL);         //OD1
  RightOdrive.begin(RIGHT_ODRIVE_SERIAL);       //OD2

  // Set ODrive Closed Loop Control Mode
  LeftOdrive.left.writeState(  AXIS_STATE_CLOSED_LOOP_CONTROL);
  LeftOdrive.right.writeState( AXIS_STATE_CLOSED_LOOP_CONTROL);
  RightOdrive.left.writeState( AXIS_STATE_CLOSED_LOOP_CONTROL);
  RightOdrive.right.writeState(AXIS_STATE_CLOSED_LOOP_CONTROL);

  //////////////////////////////////////////////////////
  RoveComm.begin(RC_DRIVEBOARD_FOURTHOCTET, &TCPServer);
  
  //after 150 seconds of no comms, disable drive
  Watchdog.begin(Estop, 150); 

  //set buttons and PWM to input
  for(int i = 0; i < 4; i++)
  {
    pinMode(motorButtons[i], INPUT);
    pinMode(encoderPins[i], INPUT);
  }

  for(int i=0; i<4; i++)
  {
    encoders[i].attach(encoderPins[i]);
    encoders[i].start();
  }
  WireBreaks.attach(T6_A);
  WireBreaks.start();
}

////////////////////////////////////////////////////////////////
void loop() {
    packet = RoveComm.read();
  
    switch(packet.data_id)
    {
      //////////////////////////////////////////////////////////
      //Read in Left/Right Wheel Speeds
      //////////////////////////////////////////////////////////
      case RC_DRIVEBOARD_DRIVELEFTRIGHT_DATA_ID:
        int16_t *leftrightspeeds;
        leftrightspeeds = (int16_t*)packet.data;

        leftspeed =  map(leftrightspeeds[0],-1000,1000,0,255);
        rightspeed = map(leftrightspeeds[1],-1000,1000,0,255);

        motorSpeeds[0] = leftspeed;   //FL
        motorSpeeds[1] = leftspeed;   //RL
        motorSpeeds[2] = rightspeed;  //FR
        motorSpeeds[3] = rightspeed;  //RR
        
        FL_SERIAL.write(leftspeed);   //FL
        RL_SERIAL.write(leftspeed);   //RL
        FR_SERIAL.write(rightspeed);  //FR
        RR_SERIAL.write(rightspeed);  //RR

        Watchdog.clearWatchdog();
        break;
      ////////////////////////////////////////////////////////
      //Read in individual speeds
      ////////////////////////////////////////////////////////
      case RC_DRIVEBOARD_DRIVEINDIVIDUAL_DATA_ID:
        int16_t *individualrcspeeds;
        individualrcspeeds = (int16_t*)packet.data;
        
        motorSpeeds[0] = individualrcspeeds[0];   //FL
        motorSpeeds[1] = individualrcspeeds[1];   //RL
        motorSpeeds[2] = individualrcspeeds[2];   //FR
        motorSpeeds[3] = individualrcspeeds[3];   //RR

        FL_SERIAL.write(motorSpeeds[0]);  //FL
        RL_SERIAL.write(motorSpeeds[1]);  //RL
        FR_SERIAL.write(motorSpeeds[2]);  //FR
        RR_SERIAL.write(motorSpeeds[3]);  //RR

        Watchdog.clearWatchdog();
        break;

      ///////////////////////////////////////////////////////////////////////
      //Set wheel angle
      ///////////////////////////////////////////////////////////////////////
      case RC_DRIVEBOARD_SETSTEERINGANGLE_DATA_ID:
        int16_t *dirAngle;
        dirAngle = (int16_t*)packet.data;   //[FL,RL,FR,RR] (0,359)

        //Re-arranging to keep consistent FL,FR,RL,RR order
        wheelAngle[0] = dirAngle[0];
        wheelAngle[1] = dirAngle[1];
        wheelAngle[2] = dirAngle[2];
        wheelAngle[3] = dirAngle[3];

        Watchdog.clearWatchdog();
        swerveDriveInit(wheelAngle);
        break;
      
      ///////////////////////////////////////////////////////////////////////
      //Watchdog Override
      ///////////////////////////////////////////////////////////////////////
      case RC_DRIVEBOARD_WATCHDOGOVERRIDE_DATA_ID:
        uint8_t *watchdogState;
        watchdogState = (uint8_t*)packet.data;  //1- Turn Off watchdog, 0- turn on
    }

    //check for button presses and override speeds if so
    for(int i = 0; i < 4; i++)
    {
      if(digitalRead(motorButtons[i]))
      {
        motorSpeeds[i] = (digitalRead(DIR_SWITCH)?1:-1)*BUTTON_OVERIDE_SPEED;
      }
    }
    for(int i = 0; i < 4; i++)
    {
      Serial.println(motorSpeeds[i]);
    }
    FL_SERIAL.write(motorSpeeds[0]); //FL
    RL_SERIAL.write(motorSpeeds[1]); //RL
    FR_SERIAL.write(motorSpeeds[2]); //FR
    RR_SERIAL.write(motorSpeeds[3]); //RR

    //If wheels move beyond the DEGREE_ALLOWABLE_DIFFERENCE during operation, then
    //swerve drive is re-initialized. THIS WILL STOP THE ROVER TO READJUST WHEELS!
    /*for(int i=0; i<4; i++)
    {  
      if(abs(encoders[i].readDegrees() - wheelAngle[i]) > DEGREE_OFFSET_TOLERANCE)
        swerveDriveInit(wheelAngle);
    }*/

}

////////////////////////////////////////////////////////
// Estop
////////////////////////////////////////////////////////
void Estop()
{
    for(int i = 0; i < 4; i++)
    {
      if(!digitalRead(motorButtons[i]))
      {
        motorSpeeds[i] = DRIVE_ZERO;
      }
    }
    Serial.println("Watchdog cleared");
    Watchdog.clearWatchdog();
}

////////////////////////////////////////////////////////
// SwerveDrive
// Called anytime wheels go beyond Degree Tolerance
// or when switching to SwerveDrive mode
////////////////////////////////////////////////////////
void swerveDriveInit(uint8_t *wheelAngle)
{
  //Ensure all motors are at DRIVE_ZERO before initiating wheel turn
  FL_SERIAL.write(DRIVE_ZERO); //FL
  RL_SERIAL.write(DRIVE_ZERO); //RL
  FR_SERIAL.write(DRIVE_ZERO); //FR
  RR_SERIAL.write(DRIVE_ZERO); //RR

  //Send move command and angle to ODrives. 
  //Function returns once wheels are at correct angle (i.e. blocks packets)
  moveWheelsToAngle(wheelAngle);

  //Update ODrive Watchdogs
  LeftOdrive.left.updateWatchdog();
  LeftOdrive.right.updateWatchdog();
  RightOdrive.left.updateWatchdog();
  RightOdrive.right.updateWatchdog();

  //Return to loop and run at given motor speeds
  //Currently, will run wheels at whatever wheel speed was last given
}

////////////////////////////////////////////////////////
// pointTurn
////////////////////////////////////////////////////////
void pointTurn(uint8_t *wheelAngle)
{
  //Prepare each wheel for point turn
  wheelAngle[0] = 45;     //FL
  wheelAngle[2] = 135;    //RL
  wheelAngle[1] = 135;    //FR
  wheelAngle[3] = 45;     //RR
  
  //Stop all motors before initiating wheel turn
  //Doesn't overwrite given wheel speed, only temporarily stops.
  FL_SERIAL.write(DRIVE_ZERO); //FL
  RL_SERIAL.write(DRIVE_ZERO); //RL
  FR_SERIAL.write(DRIVE_ZERO); //FR
  RR_SERIAL.write(DRIVE_ZERO); //RR

  //Send move command and angle to ODrives
  moveWheelsToAngle(wheelAngle);

  //Update ODrive Watchdogs
  LeftOdrive.left.updateWatchdog();
  LeftOdrive.right.updateWatchdog();
  RightOdrive.left.updateWatchdog();
  RightOdrive.right.updateWatchdog();
}

//NOTE: Rover should be stopped before initiating a wheel turn.
//      This function should be used after stoping rover.
void moveWheelsToAngle(uint8_t *goalAngle)
{
  int wheelFinishedCnt = 0;
  float curAngle[4];
  int   goalEncCnt[4];
  int   wheelSpeed[4];

  for(int i=4; i<4; i++)
  {
    curAngle[i] = encoders[i].readDegrees();
    goalEncCnt[i] = goalAngle[i] * ANGLE_TO_ENC_COUNTS;

    //(((((goalAngle[i]-curAngle[i] + 540)%360) - 180) < 0)?-1:1)*WHEEL_TURN_SPEED   -->  clockwise or counter-clock to reach angle
    wheelSpeed[i] = (((((goalAngle[i]-static_cast<int>(curAngle[i]) + 540)%360) - 180) < 0)?-1:1)*WHEEL_TURN_SPEED;

    //turn wheel if current angle is too far from desired angle
    if(abs(curAngle[i] - goalAngle[i]) > DEGREE_ALLOWABLE_INIT_DIFFERENCE)
    {
      if(i == 0)
        LeftOdrive.right.writePosSetPoint(  goalEncCnt[i], wheelSpeed[i], 0);  //FL
      if(i == 1)
        LeftOdrive.left.writePosSetPoint(   goalEncCnt[i], wheelSpeed[i], 0);  //RL
      if(i == 2)
        RightOdrive.left.writePosSetPoint(  goalEncCnt[i], wheelSpeed[i], 0);  //FR
      if(i == 3)
        RightOdrive.right.writePosSetPoint( goalEncCnt[i], wheelSpeed[i], 0);  //RR 
    }

  }
  do
  {
    for(int i=4; i<4; i++)
    {
      if(abs(curAngle[i] - goalAngle[i]) <= DEGREE_ALLOWABLE_INIT_DIFFERENCE)
      {
        if(i == 0)
          LeftOdrive.right.writeVelocitySetpoint( DRIVE_ZERO,0);  //FL
        if(i == 1)
          LeftOdrive.left.writeVelocitySetpoint(  DRIVE_ZERO,0);  //RL
        if(i == 2)
          RightOdrive.left.writeVelocitySetpoint( DRIVE_ZERO,0);  //FR
        if(i == 3)
          RightOdrive.right.writeVelocitySetpoint(DRIVE_ZERO,0);  //RR 
        
        wheelFinishedCnt++;
      }

    }

  } while (wheelFinishedCnt < 4);

}