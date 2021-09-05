//DriveBoard Software Rev 1 2021
#include "DriveBoardSoftware.h"
//192.168.1.54
/////////////////////////////////////////////////////////////////////////
EthernetServer TCPServer(RC_ROVECOMM_DRIVEBOARD_PORT);

////////////////////////////////////////////////////////////////
void setup() 
{
    // initialize all serial ports: 
    Serial.begin(9600);
    LF_SERIAL.begin(115200);                  //LF
    LR_SERIAL.begin(115200);                  //LR
    RF_SERIAL.begin(115200);                  //RF
    RR_SERIAL.begin(115200);                  //RR
    LeftOdrive.begin(  LEFT_ODRIVE_SERIAL);   //OD1
    RightOdrive.begin(RIGHT_ODRIVE_SERIAL);   //OD2
    delay(1);
    Serial.println(ANGLE_TO_ENC_COUNTS);

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
    RoveComm.begin(RC_STEERBOARD_FOURTHOCTET, &TCPServer);
    
    //after 150 seconds of no comms, disable drive
    Watchdog.beginDrive(Estop, 10000, WATCHDOG_1); 

    //set buttons and PWM to input
    for(int i = 0; i < 4; i++)
    {
        pinMode(motorButtons[i], INPUT);
    }
    pinMode(DIR_SWITCH, INPUT);
    pinMode(LFT_TURN, INPUT);
    pinMode(RHT_TURN, INPUT);

    for(int i=0; i<4; i++)
    {
        encoders[i].attach(encoderPins[i], 7, false, absoluteOffset[i], false, 0, 1000);
        encoders[i].start();
    }
}

////////////////////////////////////////////////////////////////
void loop() 
{
    packet = RoveComm.read();
    //Serial.println(packet.data_id);
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
        case RC_STEERBOARD_SETSTEERINGANGLE_DATA_ID:
            int16_t *dirAngle;
            dirAngle = (int16_t*)packet.data;   //[LF,LR,RF,RR] (0,359)

            //Re-arranging to keep consistent LF,LR,RF,RR order
            wheelAngle[0] = dirAngle[0];
            wheelAngle[1] = dirAngle[1];
            wheelAngle[2] = dirAngle[2];
            wheelAngle[3] = dirAngle[3];

            //moveWheelsToAngle(wheelAngle);
            moveWheelsToAngle(wheelAngle);
            Watchdog.clearWatchdog();
            break;

        ///////////////////////////////////////////////////////////////////////
        //Set Steering Speed
        ///////////////////////////////////////////////////////////////////////
        case RC_STEERBOARD_SETSTEERINGSPEEDS_DATA_ID:
            int16_t *steeringSpeeds;
            steeringSpeeds = (int16_t*)packet.data; //[LF,LR,RF,RR] (-1000,1000)
            LeftOdrive.left.writeControlMode(CTRL_MODE_VELOCITY_CONTROL);
            LeftOdrive.right.writeControlMode(CTRL_MODE_VELOCITY_CONTROL);
            RightOdrive.left.writeControlMode(CTRL_MODE_VELOCITY_CONTROL);
            RightOdrive.right.writeControlMode(CTRL_MODE_VELOCITY_CONTROL);

            turnSpeeds[0] = map(steeringSpeeds[0], -1000, 1000, -SWERVE_MAX_ECS, SWERVE_MAX_ECS);
            turnSpeeds[1] = map(steeringSpeeds[1], -1000, 1000, -SWERVE_MAX_ECS, SWERVE_MAX_ECS);
            turnSpeeds[2] = map(steeringSpeeds[2], -1000, 1000, -SWERVE_MAX_ECS, SWERVE_MAX_ECS);
            turnSpeeds[3] = map(steeringSpeeds[3], -1000, 1000, -SWERVE_MAX_ECS, SWERVE_MAX_ECS);
            Serial.println("Steering Speeds");
            Serial.println(turnSpeeds[0]);
            Serial.println(turnSpeeds[1]);
            Serial.println(turnSpeeds[2]);
            Serial.println(turnSpeeds[3]);

            for(int i=0; i<4; i++)
            {
                if(steeringSpeeds[i] == 0)
                {
                    turnSpeeds[i] = 0;
                }
            }
            
            LeftOdrive.right.writeVelocitySetpoint(turnSpeeds[0], 0);
            LeftOdrive.left.writeVelocitySetpoint(turnSpeeds[1], 0);
            RightOdrive.left.writeVelocitySetpoint(turnSpeeds[3], 0);
            RightOdrive.right.writeVelocitySetpoint(turnSpeeds[2], 0);

            Watchdog.clearWatchdog();
            break;

        ///////////////////////////////////////////////////////////////////////
        //PointTurn
        ///////////////////////////////////////////////////////////////////////
        case RC_STEERBOARD_POINTTURN_DATA_ID:
            int16_t* turnSpeed;
            turnSpeed = (int16_t*)packet.data;  //(-1000,1000) (Full speed CCW, full speed CW)
            pointTurn();
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

    //check for button presses and override speeds if so
    for(int i = 0; i < 4; i++)
    {
        if(digitalRead(motorButtons[i]))
        {
            motorSpeeds[i] = (digitalRead(DIR_SWITCH)?BUTTON_OVERIDE_SPEED:-BUTTON_OVERIDE_SPEED);
        }
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

    for ( uint8_t i = 0; i < 4; i++)
    {
        absoluteAngles[i] = encoders[i].readDegrees();
    }
    
    motorCurrent[0] = LeftOdrive.right.readCurrent();   //LF
    motorCurrent[1] = LeftOdrive.left.readCurrent();    //LR
    motorCurrent[2] = RightOdrive.left.readCurrent();   //RF
    motorCurrent[3] = RightOdrive.right.readCurrent();  //RR

    Serial.println("Pestimate after");
    Serial.println(LeftOdrive.left.readPosEstimate());
    Serial.println(LeftOdrive.right.readPosEstimate());
    Serial.println(RightOdrive.right.readPosEstimate());
    Serial.println(RightOdrive.left.readPosEstimate());
    //Telemetry
    if(millis() - last_update_time >= ROVECOMM_UPDATE_RATE)
    {
        //Send Drive Speeds
        //RoveComm.writeReliable(RC_DRIVEBOARD_DRIVESPEEDS_DATA_ID, RC_DRIVEBOARD_DRIVESPEEDS_DATA_COUNT, motorSpeeds);

        //Send Wheel Angles
        RoveComm.write(RC_STEERBOARD_DRIVEANGLES_DATA_ID, RC_STEERBOARD_DRIVEANGLES_DATA_COUNT, absoluteAngles);

        //Send Steering Motor Currents
        //RoveComm.write(RC_STEERBOARD_STEERINGMOTORCURRENTS_DATA_ID, RC_STEERBOARD_STEERINGMOTORCURRENTS_DATA_COUNT, motorCurrent);

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
            motorSpeeds[i] = 0;
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
void swerveDriveInit()
{
    //Ensure all motors are at 0 before initiating wheel turn
    for ( uint8_t i = 0; i < 4; i++ )
    {
        incrementAngleHome[i] = encoders[i].readDegrees();
    }

    //Return to loop and run at given motor speeds
    //Currently, will run wheels at whatever wheel speed was last given
}

////////////////////////////////////////////////////////
// pointTurn
////////////////////////////////////////////////////////
void pointTurn()
{
    //Prepare each wheel for point turn
    wheelAngle[0] = 45;     //FL
    wheelAngle[2] = 135;    //RL
    wheelAngle[1] = 135;    //FR
    wheelAngle[3] = 45;     //RR
    
    //Stop all motors before initiating wheel turn
    //Doesn't overwrite given wheel speed, only temporarily stops.
    LF_UART.setRPM((float)0);  //LF
    LR_UART.setRPM((float)0);  //LR
    RF_UART.setRPM((float)0);  //RF
    RR_UART.setRPM((float)0);  //RR

    //Send move command and angle to ODrives
    moveWheelsToAngle(wheelAngle);

    //Update ODrive Watchdogs
    LeftOdrive.left.updateWatchdog();
    LeftOdrive.right.updateWatchdog();
    RightOdrive.left.updateWatchdog();
    RightOdrive.right.updateWatchdog();
}

//NOTE: Rover should be stopped before initiating a wheel turn.
//This function should be used after stoping rover.
void moveWheelsToAngle(float *goalAngle)
{
    float goalAngleIncremental[4] = {};
    float angleThroughOrigin[4] = {};
    float angleAwayFromOrigin[4] = {};

    LeftOdrive.left.writeControlMode(CTRL_MODE_POSITION_CONTROL);
    LeftOdrive.right.writeControlMode(CTRL_MODE_POSITION_CONTROL);
    RightOdrive.left.writeControlMode(CTRL_MODE_POSITION_CONTROL);
    RightOdrive.right.writeControlMode(CTRL_MODE_POSITION_CONTROL);

    Serial.println("Pestimate before");
    Serial.println(LeftOdrive.right.readPosEstimate());
    Serial.println(LeftOdrive.left.readPosEstimate());
    Serial.println(RightOdrive.right.readPosEstimate());
    Serial.println(RightOdrive.left.readPosEstimate());

    for( int i=0; i<4; i++ )
    {
        absoluteAngles[i] = encoders[i].readDegrees();
        Serial.println("Current Angle");
        Serial.println(absoluteAngles[i]);
        angleThroughOrigin[i] = min(goalAngle[i], absoluteAngles[i]) + ( MAX_ENCODER_ANGLE - max( goalAngle[i], absoluteAngles[i] ) );
        Serial.println("AngleThroughOrigin before");
        Serial.println(angleThroughOrigin[i]);
        angleThroughOrigin[i] *= ( min(goalAngle[i], absoluteAngles[i]) == absoluteAngles[i] ? -1 : 1 );
        Serial.println("AngleThroughOrigin after");
        Serial.println(angleThroughOrigin[i]);
        angleAwayFromOrigin[i] = goalAngle[i] - absoluteAngles[i];
        Serial.println("AngleAwayFromOrigin before");
        Serial.println(angleAwayFromOrigin[i]);
        goalAngleIncremental[i] = signMin( angleThroughOrigin[i], angleAwayFromOrigin[i] ) * ANGLE_TO_ENC_COUNTS;
        Serial.println("Determined Angle ");
        Serial.println(goalAngleIncremental[i]);
        Watchdog.clearWatchdog();
    }

    LeftOdrive.left.writePosSetPoint( ( wheelDirectionFactor[1] * goalAngleIncremental[1] ) + LeftOdrive.left.readPosEstimate() , 0, 0);
    LeftOdrive.right.writePosSetPoint( ( wheelDirectionFactor[0] * goalAngleIncremental[0] ) + LeftOdrive.right.readPosEstimate(), 0, 0);
    RightOdrive.left.writePosSetPoint( ( wheelDirectionFactor[3] * goalAngleIncremental[3] ) + RightOdrive.left.readPosEstimate(), 0, 0);
    RightOdrive.right.writePosSetPoint( ( wheelDirectionFactor[2] * goalAngleIncremental[2] ) + RightOdrive.right.readPosEstimate(), 0, 0);
    Watchdog.clearWatchdog();
}

void printUARTdata(VescUart UART)
{
    Serial.println(UART.data.rpm);
    Serial.println(UART.data.inpVoltage);
    Serial.println(UART.data.ampHours);
    Serial.println(UART.data.tachometerAbs);
}

void incrementAngle(float wheelAngle)
{
    float steering_Direction = 0;

    RightOdrive.right.writeControlMode(CTRL_MODE_POSITION_CONTROL);

    steering_Direction = wheelAngle*ANGLE_TO_ENC_COUNTS;
    Serial.println(steering_Direction);

    RightOdrive.left.writePosSetPoint(steering_Direction, 0, 0);
}

float signMin(float angle, float angle2)
{
    return ( abs(angle) < abs(angle2) ? angle : angle2 );
}