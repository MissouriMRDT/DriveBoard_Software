#ifndef DriveBoardSoftware_h
#define DriveBoardSoftware_h

#include "RoveComm.h"
#include <VescUart.h>
#include "RoveWatchdog.h"
#include <math.h>

//Motor Overide Buttons
#define LF_MOTOR                PL_4
#define LR_MOTOR                PL_0
#define RF_MOTOR                PL_5
#define RR_MOTOR                PL_1
#define DIR_SWITCH              PP_2
#define BUTTON_OVERIDE_SPEED    5000

//Motor Speed Controls
#define LF_SERIAL               Serial2
#define LR_SERIAL               Serial4
#define RF_SERIAL               Serial3
#define RR_SERIAL               Serial6
#define DRIVE_MIN_RPM           0
#define DRIVE_MAX_RPM           35000

VescUart LF_UART;
VescUart LR_UART;
VescUart RF_UART;
VescUart RR_UART;

//Rovecomm
RoveCommEthernet RoveComm;
rovecomm_packet packet;
RoveWatchdog Watchdog;
uint32_t last_update_time;

//All wheels are in order of LF,LR,RF,RR
uint8_t motorButtons[4] = { LF_MOTOR, LR_MOTOR, RF_MOTOR, RR_MOTOR };
int16_t motorSpeeds[4] = {}; //LF,LR,RF,RR
float wheelAngle[4] = {}; //LF,LR,RF,RR
int32_t turnSpeeds[4] = {}; //LF,LR,RF,RR
float motorCurrent[4] = {};
uint16_t absoluteAngles[4] =  {};
float incrementAngleHome[4] = {};
float absoluteOffset[4] = { -150000, 83000, 160000, -57000};
float incrementalCWOffset[4] = {};
float incrementalCCWOffset[4] = {};
float wheelDirectionFactor[4] = { 1, -1, -1, 1 };

void EStop();
void swerveDriveInit();
void pointTurn();
void moveWheelsToAngle(float *goalAngle);
void printUARTdata(VescUart UART);
void incrementAngle(float wheelAngle);
float signMin(float angle, float angle2);

#endif
