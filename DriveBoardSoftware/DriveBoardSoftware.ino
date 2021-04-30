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
    Watchdog.beginDrive(Estop, 150, WATCHDOG_1); 

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
        encoders[i].attach(encoderPins[i]);
        encoders[i].start();
    }
}

////////////////////////////////////////////////////////////////
void loop() 
{
    packet = RoveComm.read();
    Serial.println(packet.data_id);
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

    for ( uint8_t i = 0; i < 4; i++)
    {
        absoluteAngles[i] = encoders[i].readDegrees();
    }
    
    motorCurrent[0] = LeftOdrive.right.readCurrent();   //LF
    motorCurrent[1] = LeftOdrive.left.readCurrent();    //LR
    motorCurrent[2] = RightOdrive.left.readCurrent();   //RF
    motorCurrent[3] = RightOdrive.right.readCurrent();  //RR

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
void swerveDriveInit(uint8_t *wheelAngle)
{
    //Ensure all motors are at 0 before initiating wheel turn
    LF_UART.setRPM((float)0);  //LF
    LR_UART.setRPM((float)0);  //LR
    RF_UART.setRPM((float)0);  //RF
    RR_UART.setRPM((float)0);  //RR

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
void moveWheelsToAngle(uint8_t *goalAngle)
{
    float curAngle[4] = {};
    uint16_t clockwise_distance[4] = {};
    uint16_t counterclockwise_distance[4] = {};
    int32_t steering_Direction[4] = {};

    LeftOdrive.left.writeControlMode(CTRL_MODE_POSITION_CONTROL);
    LeftOdrive.right.writeControlMode(CTRL_MODE_POSITION_CONTROL);
    RightOdrive.left.writeControlMode(CTRL_MODE_POSITION_CONTROL);
    RightOdrive.right.writeControlMode(CTRL_MODE_POSITION_CONTROL);

    for( int i=0; i<4; i++ )
    {
        curAngle[i] = encoders[i].readDegrees();
        clockwise_distance[i] = abs((goalAngle[i] - static_cast<uint16_t>(curAngle[i])) % MAX_ENCODER_ANGLE);
        counterclockwise_distance[i] = MAX_ENCODER_ANGLE - clockwise_distance[i];
        steering_Direction[i] = ( clockwise_distance[i] < counterclockwise_distance[i] ? clockwise_distance[i] : -counterclockwise_distance[i] );
        steering_Direction[i] *= ANGLE_TO_ENC_COUNTS;
    }

    LeftOdrive.left.writePosSetPoint(steering_Direction[1], 0, 0);
    LeftOdrive.right.writePosSetPoint(steering_Direction[0], 0, 0);
    RightOdrive.left.writePosSetPoint(steering_Direction[2], 0, 0);
    RightOdrive.right.writePosSetPoint(steering_Direction[3], 0, 0);
}

void printUARTdata(VescUart UART)
{
    Serial.println(UART.data.rpm);
    Serial.println(UART.data.inpVoltage);
    Serial.println(UART.data.ampHours);
    Serial.println(UART.data.tachometerAbs);
}