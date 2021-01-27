
#ifndef _DriveBoard
#define _DriveBoard

#include "RoveComm.h"
#include "RoveUsDigiMa3Pwm.h"
#include "RovesODrive.h"
#include "RoveWatchdog.h"

//Motor Overide Buttons
#define FR_MOTOR            PL_5
#define FL_MOTOR            PL_4
#define RR_MOTOR            PL_1
#define RL_MOTOR            PL_0
#define DIR_SWITCH          PP_2

#define BUTTON_OVERIDE_SPEED 140

//PWM input pins
#define FL_PWM              PF_1
#define FR_PWM              PF_2
#define RL_PWM              PF_3
#define RR_PWM              PG_0

//ODrive Serials
HardwareSerial* LEFT_ODRIVE_SERIAL = &Serial5;
HardwareSerial* RIGHT_ODRIVE_SERIAL = &Serial7;

//Motor Speed Serials
#define FL_SERIAL               Serial2
#define FR_SERIAL               Serial3
#define RL_SERIAL               Serial4
#define RR_SERIAL               Serial6

//SwerveDrive

//difference allowed before initializng driving
#define DEGREE_ALLOWABLE_INIT_DIFFERENCE   1    
//difference allowed before stoping rover mid-operation to reset wheels
#define DEGREE_ALLOWABLE_DIFFERENCE        1

//ODrives
RovesODrive LeftOdrive;
RovesODrive RightOdrive;

RoveCommEthernet RoveComm;
rovecomm_packet packet;
RoveWatchdog Watchdog;

uint8_t rightspeed;
uint8_t leftspeed;

const byte DRIVE_ZERO        = 127;
const byte ANGLE_DEFAULT     = 90;


uint8_t motorButtons[4] = {FR_MOTOR, RR_MOTOR, FL_MOTOR, RL_MOTOR};
uint8_t motorSpeeds[4]  = {DRIVE_ZERO, DRIVE_ZERO, DRIVE_ZERO, DRIVE_ZERO}; //FL, FR, RL, RR
uint8_t encoderPins[4]  = {FL_PWM, FR_PWM, RL_PWM, RR_PWM};
uint8_t wheelAngle[4]   = {ANGLE_DEFAULT, ANGLE_DEFAULT, ANGLE_DEFAULT, ANGLE_DEFAULT}; //FL, FR, RL, RR

//Drive Mode
enum DRIVE_MODE {SWERVE_DRIVE, POINT_TURN};
int driveMode = SWERVE_DRIVE;   //acts like TankDrive when at ANGLE_DEFAULT

const String encoderName[4] = {"FR Encoder",
                               "FL Encoder",
                               "RR Encoder",
                               "RL Encoder"};

void EStop();
void swerveDriveInit(uint8_t *wheelAngle);
void pointTurn(uint8_t *wheelAngle);

#endif
