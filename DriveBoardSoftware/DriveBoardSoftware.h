#ifndef DriveBoardSoftware_h
#define DriveBoardSoftware_h

#include "RoveComm.h"
#include <VescUart.h>

//Motor Overide Buttons
#define FL_MOTOR                9
#define FR_MOTOR                10
#define ML_MOTOR                11
#define MR_MOTOR                12
#define BL_MOTOR                28
#define BR_MOTOR                29
#define DIR_SWITCH              33
#define BUTTON_OVERIDE_SPEED    5000

//Motor Speed Controls
#define FL_SERIAL               Serial1
#define FR_SERIAL               Serial2
#define ML_SERIAL               Serial3
#define MR_SERIAL               Serial4
#define BL_SERIAL               Serial5
#define BR_SERIAL               Serial6
#define DRIVE_MIN_RPM           2000
#define DRIVE_MAX_RPM           30000
#define DRIVE_MAX_RAMP          120

#define TELEMETRY_UPDATE        150000
#define WATCHDOG_TIME           300000

//Vesc Serial
VescUart FL_UART;
VescUart FR_UART;
VescUart ML_UART;
VescUart MR_UART;
VescUart BL_UART;
VescUart BR_UART;

//Rovecomm
RoveCommEthernet RoveComm;
rovecomm_packet packet;
uint32_t lastUpdateTime;
uint32_t lastRampTime;
uint32_t maxRamp;
bool watchdogOverride = false;
EthernetServer TCPServer(RC_ROVECOMM_DRIVEBOARD_PORT);
IntervalTimer watchdog;
IntervalTimer telemetry;

//All wheels are in order of FL, ML, BL, FR, MR, BR
uint8_t motorButtons[6] = {FL_MOTOR, ML_MOTOR, BL_MOTOR, FR_MOTOR, MR_MOTOR, BR_MOTOR};
int16_t motorTargets[6] = {0, 0, 0, 0, 0, 0}; //FL, ML, BL, FR, MR, BR
int16_t motorSpeeds[6] = {0, 0, 0, 0, 0, 0}; //FL, ML, BL, FR, MR, BR
float motorCurrent[6] = {};

//Estop Decleration
void EStop();
void Telemetry();

#endif
