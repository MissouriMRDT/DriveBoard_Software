#ifndef DriveBoardSoftware_h
#define DriveBoardSoftware_h

#include "RoveComm.h"
#include <VescUart.h>
#include "RoveWatchdog.h"

//Motor Overide Buttons
#define FL_MOTOR                PG_1
#define FR_MOTOR                PK_4
#define ML_MOTOR                PK_5
#define MR_MOTOR                PM_0
#define BL_MOTOR                PM_1
#define BR_MOTOR                PM_2
#define DIR_SWITCH              PP_2
#define BUTTON_OVERIDE_SPEED    5000

//Motor Speed Controls
#define FL_SERIAL               Serial7
#define FR_SERIAL               Serial6
#define ML_SERIAL               Serial5
#define MR_SERIAL               Serial4
#define BL_SERIAL               Serial3
#define BR_SERIAL               Serial2
#define DRIVE_MIN_RPM           2000
#define DRIVE_MAX_RPM           14000
#define DRIVE_MAX_RAMP          28

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
RoveWatchdog Watchdog;
bool watchdogOverride;
EthernetServer TCPServer(RC_ROVECOMM_DRIVEBOARD_PORT);


//All wheels are in order of FL, ML, BL, FR, MR, BR
uint8_t motorButtons[6] = {FL_MOTOR, ML_MOTOR, BL_MOTOR, FR_MOTOR, MR_MOTOR, BR_MOTOR};
int16_t motorTargets[6] = {0, 0, 0, 0, 0, 0}; //FL, ML, BL, FR, MR, BR
int16_t motorSpeeds[6] = {0, 0, 0, 0, 0, 0}; //FL, ML, BL, FR, MR, BR
float motorCurrent[6] = {};

//Estop Decleration
void EStop();


#endif
