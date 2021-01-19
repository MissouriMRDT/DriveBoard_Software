
#ifndef _DriveBoard
#define _DriveBoard

#include "RoveComm.h"
#include "RoveWatchdog.h"

#define FR_MOTOR            PL_5
#define FL_MOTOR            PL_4
#define RR_MOTOR            PL_1
#define RL_MOTOR            PL_0

RoveCommEthernet RoveComm;
rovecomm_packet packet;
RoveWatchdog Watchdog;

uint8_t rightspeed;
uint8_t leftspeed;

const byte DRIVE_ZERO        = 127;


uint8_t motorButtons[4] = {FR_MOTOR, RR_MOTOR, FL_MOTOR, RL_MOTOR};
uint8_t motorSpeeds[4] = {DRIVE_ZERO, DRIVE_ZERO, DRIVE_ZERO, DRIVE_ZERO}; //FR, RR, FL, RL


void EStop();

#endif
