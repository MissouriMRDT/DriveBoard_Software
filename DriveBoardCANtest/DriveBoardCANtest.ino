//DriveBoard CAN Communication Test

#include "DriveBoardSoftware.h"
#include "driverlib/can.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"

unsigned int buffer;                                    //4 byte message data
unsigned char *bufferPtr = (unsigned char *)&buffer;
tCANMsgObject sMsgObjectRx;
tCANMsgObject sMsgObjectTx;

void setup()
{
    Serial.begin(9600);
    
    tCANBitClkParms CANBitClk;

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
}

void loop()
{
    //TEST TRANSMISSION!
    CANMessageSet(CAN0_BASE, 1, &sMsgObjectTx, MSG_OBJ_TYPE_TX);  //send as msg object 1 

}