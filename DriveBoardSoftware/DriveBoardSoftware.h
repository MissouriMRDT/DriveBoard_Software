#ifndef DriveBoardSoftware_h
#define DriveBoardSoftware_h

#include "RoveComm.h"
#include <VescUart.h>
#include "RoveUsDigiMa3Pwm.h"
#include "RovesODrive.h"
#include "RoveWatchdog.h"
#include <math.h>

//Motor Overide Buttons
#define LF_MOTOR                PL_4
#define LR_MOTOR                PL_0
#define RF_MOTOR                PL_5
#define RR_MOTOR                PL_1
#define DIR_SWITCH              PP_2
#define BUTTON_OVERIDE_SPEED    5000

#define LFT_TURN                PL_2
#define RHT_TURN                PL_3

//Encoder PWM input pins
#define LF_PWM                  PD_0
#define LR_PWM                  PD_1
#define RF_PWM                  PM_6
#define RR_PWM                  PM_7

//SwerveDrive
#define WHEEL_TURN_SPEED        2000 //RPM
#define DEGREE_OFFSET_TOLERANCE 1    //stops rover to readjust wheels is above tolerance 
#define LF_ABSOLUTE_OFFSET      62
#define LR_ABSOLUTE_OFFSET      62
#define RF_ABSOLUTE_OFFSET      62
#define RR_ABSOLUTE_OFFSET      62

//Motor Speed Controls
#define LF_SERIAL               Serial2
#define LR_SERIAL               Serial4
#define RF_SERIAL               Serial3
#define RR_SERIAL               Serial6
#define DRIVE_MIN_RPM           0
#define DRIVE_MAX_RPM           35000

// Encoder values
#define ENC_CPR                 8192
#define GEAR_RATIO              700
#define MAX_ENCODER_ANGLE       360
#define ANGLE_TO_ENC_COUNTS     ((ENC_CPR * GEAR_RATIO) / (MAX_ENCODER_ANGLE))

//Odrive Velocity
#define SWERVE_MIN_ECS          0
#define SWERVE_MAX_ECS          ANGLE_TO_ENC_COUNTS*120

VescUart LF_UART;
VescUart LR_UART;
VescUart RF_UART;
VescUart RR_UART;

//ODrive Serials
HardwareSerial* LEFT_ODRIVE_SERIAL = &Serial5;
HardwareSerial* RIGHT_ODRIVE_SERIAL = &Serial7;

//ODrives
RovesODrive LeftOdrive;
RovesODrive RightOdrive;
int16_t rightspeed;
int16_t leftspeed;

//Rovecomm
RoveCommEthernet RoveComm;
rovecomm_packet packet;
RoveWatchdog Watchdog;
uint32_t last_update_time;

//All wheels are in order of LF,LR,RF,RR
uint8_t motorButtons[4] = { LF_MOTOR, LR_MOTOR, RF_MOTOR, RR_MOTOR };
int16_t motorSpeeds[4] = {}; //LF,LR,RF,RR
uint8_t encoderPins[4] = { LF_PWM, LR_PWM, RF_PWM, RR_PWM };
float wheelAngle[4] = {}; //LF,LR,RF,RR
int32_t turnSpeeds[4] = {}; //LF,LR,RF,RR
float motorCurrent[4] = {};
uint16_t absoluteAngles[4] =  {};
float incrementAngleHome[4] = {};
float absoluteOffset[4] = {0, 0, 0, 62};
float incrementalCWOffset[4] = {};
float incrementalCCWOffset[4] = {};

RoveUsDigiMa3Pwm EncoderLF, EncoderLR, EncoderRF, EncoderRR;
RoveUsDigiMa3Pwm encoders[4] = { EncoderLF, EncoderLR, EncoderRF, EncoderRR };
RoveUsDigiMa3PwmWireBreaks  WireBreaks;

void EStop();
void swerveDriveInit();
void pointTurn();
void moveWheelsToAngle(float *goalAngle);
void printUARTdata(VescUart UART);
void incrementAngle(float wheelAngle);

#endif
