
#ifndef _DriveBoard
#define _DriveBoard

#include "RoveComm.h"
#include "RoveWatchdog.h"
#include <SPI.h>

#define FR_MOTOR            PF_3
#define FL_MOTOR            PG_0
#define MR_MOTOR            PL_5
#define ML_MOTOR            PL_0
#define RR_MOTOR            PL_2
#define RL_MOTOR            PL_3
#define SLAVE_SELECT        PD_2

RoveCommEthernet RoveComm;
rovecomm_packet packet;
RoveWatchdog Watchdog;
EthernetServer TCPServer(RC_ROVECOMM_DRIVEBOARD_PORT);

uint8_t rightspeed;
uint8_t leftspeed;

const byte DRIVE_ZERO        = 127;


uint8_t motorButtons[6] = {FR_MOTOR, MR_MOTOR, RR_MOTOR, FL_MOTOR, ML_MOTOR, RL_MOTOR};
uint8_t motorSpeeds[6] = {DRIVE_ZERO, DRIVE_ZERO, DRIVE_ZERO, DRIVE_ZERO, DRIVE_ZERO, DRIVE_ZERO}; //FR, MR, RR, FL, ML, RL


void EStop();

#endif
