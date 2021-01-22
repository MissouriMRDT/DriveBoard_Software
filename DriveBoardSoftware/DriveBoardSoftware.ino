//DriveBoard Software Rev 1 2021

#include "DriveBoardSoftware.h"
#include "driverlib/can.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"

EthernetServer TCPServer(RC_ROVECOMM_ETHERNET_DRIVE_LIGHTING_BOARD_PORT);

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

  //CAN Setup
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

  RoveComm.begin(RC_DRIVEBOARD_FOURTHOCTET, &TCPServer);
  
  //after 150 seconds of no comms, disable drive
  Watchdog.begin(Estop, 150); 

  //set buttons and PWM to input
  for(int i = 0; i < 4; i++)
  {
    pinMode(motorButtons[i], INPUT);
    pinMode(pwmEncoders[i], INPUT);
  }

  //set PWM pins to input
}

void loop() {
    packet = RoveComm.read();
  
    switch(packet.data_id)
    {
      //read in a left and right speed
      case RC_DRIVEBOARD_DRIVELEFTRIGHT_DATAID:
        int16_t *leftrightspeeds;
        leftrightspeeds = (int16_t*)packet.data;

        rightspeed = map(leftrightspeeds[0],-1000,1000,0,255);
        leftspeed = map(-leftrightspeeds[1],-1000,1000,0,255);

        motorSpeeds[0] = leftspeed;
        motorSpeeds[1] = rightspeed;
        motorSpeeds[2] = leftspeed;
        motorSpeeds[3] = rightspeed;
        
        Watchdog.clear();
        break;
      //read in individual speeds
      case RC_DRIVEBOARD_DRIVEINDIVIDUAL_DATAID:
        int16_t *individualrcspeeds;
        individualrcspeeds = (int16_t*)packet.data;
        Watchdog.clear();
        break;

      //Initialize SwerveDrive (currently the RoveCommManifest.h does not include dataID for swerve features)
      //Pass data to ODrives
      /*case RC_DRIVEBOARD_SWERVELEFTRIGHT_DATAID
        uint16_t* dirAngle = (uint8_t*)packet.data;   //[LF,LR,RF,RR] (0,359)
        swerveDriveInit(dirAngle);
        break;*/
    }

    //check for button presses and override speeds if so
    for(int i = 0; i < 4; i++)
    {
      if(digitalRead(motorButtons[i]))
      {
        motorSpeeds[i] = 140;
      }
    }
    for(int i = 0; i < 4; i++)
    {
      Serial.println(motorSpeeds[i]);
    }
    Serial2.write(motorSpeeds[0]); //FL
    Serial3.write(motorSpeeds[1]); //FR
    Serial4.write(motorSpeeds[2]); //BL
    Serial6.write(motorSpeeds[3]); //BR
}

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
    Watchdog.clear();
}

void swerveDriveInit(uint16_t dirAngle)
{
  //Ensure all motors are at DRIVE_ZERO before initiating wheel turn
  for(int i=0; i<4; i++)
    motorSpeeds[i] = DRIVE_ZERO;

  Serial2.write(motorSpeeds[0]); //FL
  Serial3.write(motorSpeeds[1]); //FR
  Serial4.write(motorSpeeds[2]); //BL
  Serial6.write(motorSpeeds[3]); //BR
  delay(500);   
    
  //send directional angle to ODrives 
  Serial5.write(dirAngle);
  Serial7.write(dirAngle);

  //read value from encoder (10-bit) and map value to angle
  for(int i=0; i<4; i++)
  {
    encoderAngle[i] = map(pulseIn(pwmEncoders[i], HIGH), 0, 1023, 0, 359);
  }
  
}
