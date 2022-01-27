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
#define DRIVE_MIN_RPM           0
#define DRIVE_MAX_RPM           14000

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
RoveWatchdog Watchdog;
uint32_t last_update_time;

//All wheels are in order of FL, FR, ML, MR, BL, BR
uint8_t motorButtons[6] = {FL_MOTOR, FR_MOTOR, ML_MOTOR, MR_MOTOR, BL_MOTOR, BR_MOTOR};
int16_t motorSpeeds[6] = {}; //FL, FR, ML, MR, BL, BR
float motorCurrent[6] = {};

//Estop Decleration
void EStop();


#endif
