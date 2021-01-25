 //DriveBoard Software Rev 1 2021

#include "DriveBoardSoftware.h"
#include "driverlib/can.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"

/////////////////////////////////////////////////////////////////////////
EthernetServer TCPServer(RC_ROVECOMM_ETHERNET_DRIVE_LIGHTING_BOARD_PORT);

/////////////////////////////////////////////
RoveUsDigiMa3Pwm EncoderFR, EncoderFL, EncoderRR, EncoderRL;
RoveUsDigiMa3Pwm encoders[4]=    {EncoderFR,
                                  EncoderFL,
                                  EncoderRR,
                                  EncoderRL};
RoveUsDigiMa3PwmWireBreaks  WireBreaks;


//////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  // initialize all serial ports: 
  Serial.begin(19200);
  // Motor speed serial ports
  Serial2.begin(19200);       //FL
  Serial3.begin(19200);       //FR
  Serial4.begin(19200);       //BL
  Serial6.begin(19200);       //BR
  // ODrive Serial Ports
  Serial5.begin(19200);       //OD1
  Serial7.begin(19200);       //OD2

  ///////////////////////////////////////////////////////////////////////////////////////////////
  // CAN ODrive Communication
  //////////////////////////////////////////////////////////////////////////////////////////////
  /*tCANBitClkParms CANBitClk;
  tCANMsgObject sMsgObjectRx;
  tCANMsgObject sMsgObjectTx;
  unsigned int buffer;                                    //4 byte message data
  unsigned char *bufferPtr = (unsigned char *)&buffer;    //access individual bytes using pointer

  SysCtlPeripheralEnable(SYSCTL_PERIPH_CAN0);             //Enable the CAN0 module
  CANInit(CAN0_BASE);                                     //initialize CAN0 module
  CANBitRateSet(CAN0_BASE, SysCtlClockGet(), 1000000u);   //Configure controller for 1Mbit operation (MCP2551 transceiver supports up to 1Mb/s)
  CANEnable(CAN0_BASE);                                   //Take CAN0 out of INIT state

  //TEST MSG
  bufferPtr[0] = 128;
  bufferPtr[1] = 127;
  bufferPtr[2] = 126;
  bufferPtr[3] = 125;

  //CAN Transmit Configuration
  sMsgObjectTx.ui32MsgID = 0x400u;
  sMsgObjectTx.ui32Flags = 0u;
  sMsgObjectTx.ui32MsgLen = sizeof(bufferPtr);  //4 byte max message
  sMsgObjectTx.pui8MsgData = bufferPtr;

  //TEST TRANSMISSION!
  CANMessageSet(CAN0_BASE, 1, &sMsgObjectTx, MSG_OBJ_TYPE_TX);  //send as msg object 1   */

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

////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {
    packet = RoveComm.read();
  
    switch(packet.data_id)
    {
      ////////////////////////////////////////////////////////
      //read in a left and right speed
      ////////////////////////////////////////////////////////
      case RC_DRIVEBOARD_DRIVELEFTRIGHT_DATAID:
        int16_t *leftrightspeeds;
        leftrightspeeds = (int16_t*)packet.data;

        rightspeed = map(leftrightspeeds[0],-1000,1000,0,255);
        leftspeed = map(-leftrightspeeds[1],-1000,1000,0,255);

        motorSpeeds[0] = leftspeed;
        motorSpeeds[1] = rightspeed;
        motorSpeeds[2] = leftspeed;
        motorSpeeds[3] = rightspeed;
        
        Watchdog.clearWatchdog();
        break;
      ////////////////////////////////////////////////////////
      //read in individual speeds
      ////////////////////////////////////////////////////////
      case RC_DRIVEBOARD_DRIVEINDIVIDUAL_DATAID:
        int16_t *individualrcspeeds;
        individualrcspeeds = (int16_t*)packet.data;
        Watchdog.clearWatchdog();
        break;

      ///////////////////////////////////////////////////////////////////////
      //Initialize SwerveDrive (currently the RoveCommManifest.h does not include dataID for swerve features)
      //Pass data to ODrives
      ///////////////////////////////////////////////////////////////////////
      /*case RC_DRIVEBOARD_SWERVELEFTRIGHT_DATAID
          uint16_t *dirAngle
          dirAngle = (uint16_t*)packet.data;   //[LF,LR,RF,RR] (0,359)

          //Re-arranging to keep consistent FL,FR,RL,RR order
          wheelAngle[0] = dirAngle[0];
          wheelAngle[1] = dirAngle[2];
          wheelAngle[2] = dirAngle[1];
          wheelAngle[3] = dirAngle[3];
          swerveDriveInit(wheelAngle);
          break;*/
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
    Serial2.write(motorSpeeds[0]); //FL
    Serial3.write(motorSpeeds[1]); //FR
    Serial4.write(motorSpeeds[2]); //RL
    Serial6.write(motorSpeeds[3]); //RR
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
////////////////////////////////////////////////////////
void swerveDriveInit(uint8_t wheelAngle)
{
  //Ensure all motors are at DRIVE_ZERO before initiating wheel turn
  for(int i=0; i<4; i++)
    motorSpeeds[i] = DRIVE_ZERO;

  FL_SERIAL.write(motorSpeeds[0]); //FL
  FR_SERIAL.write(motorSpeeds[1]); //FR
  RL_SERIAL.write(motorSpeeds[2]); //RL
  RR_SERIAL.write(motorSpeeds[3]); //RR
    
  //send directional angle to ODrives 
  Serial5.write(wheelAngle);
  Serial7.write(wheelAngle);

  //read value from encoder and print to serial monitor
  for(int i=0; i<4; i++)
  {
    Serial.println();
    Serial.print(encoderName[i]);     Serial.println("////////////////");
    Serial.print("Millidegrees:  ");  Serial.print(encoders[i].readMillidegrees()); Serial.println(" millidegrees");
    Serial.print("Degrees:       ");  Serial.print(encoders[i].readDegrees()     ); Serial.println(" degrees");
    Serial.print("Radians:       ");  Serial.print(encoders[i].readRadians()     ); Serial.println(" radians");
    Serial.println();
  }
}

////////////////////////////////////////////////////////
// pointTurn
////////////////////////////////////////////////////////
void pointTurn(uint8_t *wheelAngle)
{
  int maxDegreeDifference = DEGREE_ALLOWABLE_DIFFERENCE + 1;
  
  //Prepare each wheel for point turn
  wheelAngle[0] = 45;     //FL
  wheelAngle[1] = 135;    //FR
  wheelAngle[2] = 135;    //RL
  wheelAngle[3] = 45;     //RR
  
  //Ensure all motors are at DRIVE_ZERO before initiating wheel turn
  FL_SERIAL.write(DRIVE_ZERO); //FL
  FR_SERIAL.write(DRIVE_ZERO); //FR
  RL_SERIAL.write(DRIVE_ZERO); //RL
  RR_SERIAL.write(DRIVE_ZERO); //RR

  //send directional angle to ODrives 
  LEFT_ODRIVE_SERIAL.write(wheelAngle[0]);       //ODrive on left wheels
  LEFT_ODRIVE_SERIAL.write(',');
  LEFT_ODRIVE_SERIAL.write(wheelAngle[2]);
  
  RIGHT_ODRIVE_SERIAL.write(wheelAngle[1]);       //ODrive on right wheels
  RIGHT_ODRIVE_SERIAL.write(',');
  RIGHT_ODRIVE_SERIAL.write(wheelAngle[3]);

  //wait until all wheels are within allowable difference range before driving
  while(maxDegreeDifference > DEGREE_ALLOWABLE_DIFFERENCE)
  {
    for(int i=0; i<4; i++)     
      maxDegreeDifference = abs(encoders[i].readDegrees() - wheelAngle[i]);
  }

  /*
  if(turnSpeed < 0)                     //counter-clockwise
  {
    FL_SERIAL.write(motorSpeeds[0]); //FL
    FR_SERIAL.write(-motorSpeeds[1]); //FR
    RL_SERIAL.write(-motorSpeeds[2]); //RL
    RR_SERIAL.write(motorSpeeds[3]); //RR
  }

  if(turnSpeed > 0)                     //clockwise
  {
    FL_SERIAL.write(-motorSpeeds[0]); //FL
    FR_SERIAL.write(motorSpeeds[1]); //FR
    RL_SERIAL.write(motorSpeeds[2]); //RL
    RR_SERIAL.write(-motorSpeeds[3]); //RR
  }
   */

}
  
