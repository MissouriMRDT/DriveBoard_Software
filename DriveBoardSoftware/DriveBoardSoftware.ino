//DriveBoard Software Rev 1 2021

#include "DriveBoardSoftware.h"
//192.168.1.50
/////////////////////////////////////////////////////////////////////////
EthernetServer TCPServer(RC_ROVECOMM_DRIVEBOARD_PORT);

////////////////////////////////////////////////////////////////
void setup() {
  // initialize all serial ports: 
  Serial.begin(9600);
  LF_SERIAL.begin(115200);                  //LF
  LR_SERIAL.begin(115200);                  //LR
  RF_SERIAL.begin(115200);                  //RF
  RR_SERIAL.begin(115200);                  //RR
  LeftOdrive.begin(  LEFT_ODRIVE_SERIAL);   //OD1
  RightOdrive.begin(RIGHT_ODRIVE_SERIAL);   //OD2
  delay(1);

  //Initiate the VescUart
  LF_UART.setSerialPort(&LF_SERIAL);
  LR_UART.setSerialPort(&LR_SERIAL);
  RF_UART.setSerialPort(&RF_SERIAL);
  RR_UART.setSerialPort(&RR_SERIAL);

  delay(10000);
  // Set ODrive Closed Loop Control Mode
  LeftOdrive.left.writeState(  AXIS_STATE_CLOSED_LOOP_CONTROL);
  LeftOdrive.right.writeState( AXIS_STATE_CLOSED_LOOP_CONTROL);
  RightOdrive.left.writeState( AXIS_STATE_CLOSED_LOOP_CONTROL);
  RightOdrive.right.writeState(AXIS_STATE_CLOSED_LOOP_CONTROL);

  //////////////////////////////////////////////////////
  RoveComm.begin(RC_DRIVEBOARD_FOURTHOCTET, &TCPServer);
  
  //after 150 seconds of no comms, disable drive
  //Watchdog.beginDrive(Estop, 150, WATCHDOG_1); 

  //set buttons and PWM to input
  for(int i = 0; i < 4; i++)
  {
    pinMode(motorButtons[i], INPUT);
    pinMode(encoderPins[i], INPUT);
  }
  pinMode(DIR_SWITCH, INPUT);
  pinMode(LFT_TURN, INPUT);
  pinMode(RHT_TURN, INPUT);

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

        leftspeed =  map(leftrightspeeds[0],-1000,1000,DRIVE_MIN_RPM,DRIVE_MAX_RPM);
        rightspeed = map(leftrightspeeds[1],-1000,1000,DRIVE_MIN_RPM,DRIVE_MAX_RPM);

        motorSpeeds[0] = leftspeed;   //LF
        motorSpeeds[1] = leftspeed;   //LR
        motorSpeeds[2] = rightspeed;  //RF
        motorSpeeds[3] = rightspeed;  //RR
        
        LF_UART.setRPM((float)motorSpeeds[0]);  
        LR_UART.setRPM((float)motorSpeeds[1]);  
        RF_UART.setRPM((float)motorSpeeds[2]);  
        RR_UART.setRPM((float)motorSpeeds[3]);  

        Watchdog.clearWatchdog();
        break;
      ////////////////////////////////////////////////////////
      //Read in individual speeds
      ////////////////////////////////////////////////////////
      case RC_DRIVEBOARD_DRIVEINDIVIDUAL_DATA_ID:
        int16_t *individualrcspeeds;
        individualrcspeeds = (int16_t*)packet.data;
        
        motorSpeeds[0] = map(individualrcspeeds[0], -1000, 1000, DRIVE_MIN_RPM, DRIVE_MAX_RPM);   //LF
        motorSpeeds[1] = map(individualrcspeeds[1], -1000, 1000, DRIVE_MIN_RPM, DRIVE_MAX_RPM);   //LR
        motorSpeeds[2] = map(individualrcspeeds[2], -1000, 1000, DRIVE_MIN_RPM, DRIVE_MAX_RPM);   //RF
        motorSpeeds[3] = map(individualrcspeeds[3], -1000, 1000, DRIVE_MIN_RPM, DRIVE_MAX_RPM);   //RR

        LF_UART.setRPM((float)motorSpeeds[0]);  //LF
        LR_UART.setRPM((float)motorSpeeds[1]);  //LR
        RF_UART.setRPM((float)motorSpeeds[2]);  //RF
        RR_UART.setRPM((float)motorSpeeds[3]);  //RR

        Watchdog.clearWatchdog();
        break;

      ///////////////////////////////////////////////////////////////////////
      //Set wheel angle
      ///////////////////////////////////////////////////////////////////////
      case RC_DRIVEBOARD_SETSTEERINGANGLE_DATA_ID:
        int16_t *dirAngle;
        dirAngle = (int16_t*)packet.data;   //[LF,LR,RF,RR] (0,359)

        //Re-arranging to keep consistent LF,LR,RF,RR order
        wheelAngle[0] = dirAngle[0];
        wheelAngle[1] = dirAngle[1];
        wheelAngle[2] = dirAngle[2];
        wheelAngle[3] = dirAngle[3];

        Watchdog.clearWatchdog();
        swerveDriveInit(wheelAngle);
        break;

      ///////////////////////////////////////////////////////////////////////
      //Set Steering Speed
      ///////////////////////////////////////////////////////////////////////
      case RC_DRIVEBOARD_SETSTEERINGSPEEDS_DATA_ID:
        int16_t *steeringSpeeds;
        steeringSpeeds = (int16_t*)packet.data; //[LF,LR,RF,RR] (-1000,1000)

        turnSpeeds[0] = map(steeringSpeeds[0], -1000, 1000, -SWERVE_MAX_ECS, SWERVE_MAX_ECS);
        turnSpeeds[1] = map(steeringSpeeds[1], -1000, 1000, -SWERVE_MAX_ECS, SWERVE_MAX_ECS);
        turnSpeeds[2] = map(steeringSpeeds[2], -1000, 1000, -SWERVE_MAX_ECS, SWERVE_MAX_ECS);
        turnSpeeds[3] = map(steeringSpeeds[3], -1000, 1000, -SWERVE_MAX_ECS, SWERVE_MAX_ECS);

        LeftOdrive.right.writeVelocitySetpoint(turnSpeeds[0], 0);
        LeftOdrive.left.writeVelocitySetpoint(turnSpeeds[1], 0);
        RightOdrive.left.writeVelocitySetpoint(turnSpeeds[2], 0);
        RightOdrive.right.writeVelocitySetpoint(turnSpeeds[3], 0);

        Watchdog.clearWatchdog();
        break;

      ///////////////////////////////////////////////////////////////////////
      //PointTurn
      ///////////////////////////////////////////////////////////////////////
      case RC_DRIVEBOARD_POINTTURN_DATA_ID:
        int16_t* turnSpeed;
        turnSpeed = (int16_t*)packet.data;  //(-1000,1000) (Full speed CCW, full speed CW)
        
        //WheelAngle is specified sperately from PoinTurn packet
        //set left/right velocity in reverse depepnding on CW or CCW direction

        motorSpeeds[0] = map(turnSpeed[0] <= 0? -turnSpeed[0]: turnSpeed[0], -1000, 1000, DRIVE_MIN_RPM, DRIVE_MAX_RPM);   //LF
        motorSpeeds[1] = map(turnSpeed[0] <= 0? -turnSpeed[0]: turnSpeed[0], -1000, 1000, DRIVE_MIN_RPM, DRIVE_MAX_RPM);   //LR
        motorSpeeds[2] = map(turnSpeed[0] <= 0? turnSpeed[0]: -turnSpeed[0], -1000, 1000, DRIVE_MIN_RPM, DRIVE_MAX_RPM);   //RF
        motorSpeeds[3] = map(turnSpeed[0] <= 0? turnSpeed[0]: -turnSpeed[0], -1000, 1000, DRIVE_MIN_RPM, DRIVE_MAX_RPM);   //RR

        Watchdog.clearWatchdog();
        break;

      ///////////////////////////////////////////////////////////////////////
      //Watchdog Override
      ///////////////////////////////////////////////////////////////////////
      case RC_DRIVEBOARD_WATCHDOGOVERRIDE_DATA_ID:
        uint8_t *watchdogState;
        watchdogState = (uint8_t*)packet.data;  //1- Turn Off watchdog, 0- turn on
        break;
    }

    if(LF_UART.getVescValues())
      printUARTdata(LF_UART);
    else
      Serial.println("ERROR: Can't find FL_UART");

    if(LR_UART.getVescValues())
      printUARTdata(LR_UART);
    else
      Serial.println("ERROR: Can't find RL_UART");

    if(RF_UART.getVescValues())
      printUARTdata(RF_UART);
    else
      Serial.println("ERROR: Can't find FR_UART");

    if(RR_UART.getVescValues())
      printUARTdata(RR_UART);
    else
      Serial.println("ERROR: Can't find RR_UART");

    //check for button presses and override speeds if so
    for(int i = 0; i < 4; i++)
    {
      if(digitalRead(motorButtons[i]))
      {
        motorSpeeds[i] = (digitalRead(DIR_SWITCH)?BUTTON_OVERIDE_SPEED:-BUTTON_OVERIDE_SPEED);
      }
    }

    for(int i = 0; i < 4; i++)
    {
      Serial.println(motorSpeeds[i]);
    }

    LF_UART.setRPM((float)motorSpeeds[0]);  //LF
    LR_UART.setRPM((float)motorSpeeds[1]);  //LR
    RF_UART.setRPM((float)motorSpeeds[2]);  //RF
    RR_UART.setRPM((float)motorSpeeds[3]);  //RR*/

    //If wheels move beyond the DEGREE_ALLOWABLE_DIFFERENCE during operation, then
    //swerve drive is re-initialized. THIS WILL STOP THE ROVER TO READJUST WHEELS!
    /*for(int i=0; i<4; i++)
    {  
      if(abs(encoders[i].readDegrees() - wheelAngle[i]) > DEGREE_OFFSET_TOLERANCE)
        swerveDriveInit(wheelAngle);
    }*/

    if(digitalRead(LFT_TURN))
    {
        LeftOdrive.right.writeVelocitySetpoint(20000, 0);
        LeftOdrive.left.writeVelocitySetpoint(20000, 0);
        RightOdrive.left.writeVelocitySetpoint(20000, 0);
        RightOdrive.right.writeVelocitySetpoint(20000, 0);
        Serial.println("LFT_TURN BUTTON PRESSED");
    }


    motorCurrent[0] = LeftOdrive.right.readCurrent();   //LF
    motorCurrent[1] = LeftOdrive.left.readCurrent();    //LR
    motorCurrent[2] = RightOdrive.left.readCurrent();   //RF
    motorCurrent[3] = RightOdrive.right.readCurrent();  //RR

    //Telemetry
    if(millis() - last_update_time >= ROVECOMM_UPDATE_RATE)
    {
      //Send Drive Speeds
      RoveComm.writeReliable(RC_DRIVEBOARD_DRIVESPEEDS_DATA_ID,
                             RC_DRIVEBOARD_DRIVESPEEDS_DATA_COUNT,
                             motorSpeeds);

      //Send Wheel Angles
      RoveComm.writeReliable(RC_DRIVEBOARD_DRIVEANGLES_DATA_ID,
                             RC_DRIVEBOARD_DRIVEANGLES_DATA_COUNT,
                             wheelAngle);

      //Send Steering Motor Currents
      RoveComm.writeReliable(RC_DRIVEBOARD_STEERINGMOTORCURRENTS_DATA_ID,
                             RC_DRIVEBOARD_STEERINGMOTORCURRENTS_DATA_COUNT,
                             motorCurrent);

      last_update_time = millis();
    }

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
    LeftOdrive.right.writeVelocitySetpoint(0, 0);
    LeftOdrive.left.writeVelocitySetpoint(0, 0);
    RightOdrive.left.writeVelocitySetpoint(0, 0);
    RightOdrive.right.writeVelocitySetpoint(0, 0);

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
  LF_UART.setRPM((float)DRIVE_ZERO);  //LF
  LR_UART.setRPM((float)DRIVE_ZERO);  //LR
  RF_UART.setRPM((float)DRIVE_ZERO);  //RF
  RR_UART.setRPM((float)DRIVE_ZERO);  //RR

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
  LF_UART.setRPM((float)DRIVE_ZERO);  //LF
  LR_UART.setRPM((float)DRIVE_ZERO);  //LR
  RF_UART.setRPM((float)DRIVE_ZERO);  //RF
  RR_UART.setRPM((float)DRIVE_ZERO);  //RR

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

  for(int i=4; i<4; i++)
  {
    curAngle[i] = encoders[i].readDegrees();
    goalEncCnt[i] = goalAngle[i] * ANGLE_TO_ENC_COUNTS;

    //(((((goalAngle[i]-curAngle[i] + 540)%360) - 180) < 0)?-1:1)*WHEEL_TURN_SPEED   -->  clockwise or counter-clock to reach angle
    turnSpeeds[i] = (((((goalAngle[i]-static_cast<int>(curAngle[i]) + 540)%360) - 180) < 0)?-1:1)*turnSpeeds[i];

    //turn wheel if current angle is too far from desired angle
    if(abs(curAngle[i] - goalAngle[i]) > DEGREE_ALLOWABLE_INIT_DIFFERENCE)
    {
      if(i == 0)
        LeftOdrive.right.writePosSetPoint(  goalEncCnt[i], turnSpeeds[i], 0);  //FL
      if(i == 1)
        LeftOdrive.left.writePosSetPoint(   goalEncCnt[i], turnSpeeds[i], 0);  //RL
      if(i == 2)
        RightOdrive.left.writePosSetPoint(  goalEncCnt[i], turnSpeeds[i], 0);  //FR
      if(i == 3)
        RightOdrive.right.writePosSetPoint( goalEncCnt[i], turnSpeeds[i], 0);  //RR 
    }

  }
  do
  {
    for(int i=4; i<4; i++)
    {
      if(abs(curAngle[i] - goalAngle[i]) <= DEGREE_ALLOWABLE_INIT_DIFFERENCE)
      {
        if(i == 0)
          LeftOdrive.right.writeVelocitySetpoint( SWERVE_MIN_ECS,0);  //FL
        if(i == 1)
          LeftOdrive.left.writeVelocitySetpoint(  SWERVE_MIN_ECS,0);  //RL
        if(i == 2)
          RightOdrive.left.writeVelocitySetpoint( SWERVE_MIN_ECS,0);  //FR
        if(i == 3)
          RightOdrive.right.writeVelocitySetpoint(SWERVE_MIN_ECS,0);  //RR 
        
        wheelFinishedCnt++;
      }

    }

  } while (wheelFinishedCnt < 4);

}

void printUARTdata(VescUart UART)
{
  Serial.println(UART.data.rpm);
  Serial.println(UART.data.inpVoltage);
  Serial.println(UART.data.ampHours);
  Serial.println(UART.data.tachometerAbs);
}
