
#ifndef _DriveBoard
#define _DriveBoard

#include "RoveComm.h"
#include <VescUart.h>
#include "RoveUsDigiMa3Pwm.h"
#include "RovesODrive.h"
#include "RoveWatchdog.h"

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
#define LF_PWM              PF_1
#define LR_PWM              PF_3
#define RF_PWM              PF_2
#define RR_PWM              PG_0

//SwerveDrive
#define WHEEL_TURN_SPEED                    2000 //RPM
#define DEGREE_ALLOWABLE_INIT_DIFFERENCE    1    //Initialize wheel turn when within this degree difference
#define DEGREE_OFFSET_TOLERANCE             1    //stops rover to readjust wheels is above tolerance 

//Motor Speed Controls
#define LF_SERIAL               Serial2
#define LR_SERIAL               Serial4
#define RF_SERIAL               Serial3
#define RR_SERIAL               Serial6

VescUart LF_UART;
VescUart LR_UART;
VescUart RF_UART;
VescUart RR_UART;

const int DRIVE_MIN_RPM = 0;
const int DRIVE_MAX_RPM = 35000;

//ODrive Serials
HardwareSerial* LEFT_ODRIVE_SERIAL = &Serial5;
HardwareSerial* RIGHT_ODRIVE_SERIAL = &Serial7;

//ODrives
RovesODrive LeftOdrive;
RovesODrive RightOdrive;

const int ENC_CPR               = 8192;
const int GEAR_RATIO            = 700;
const float MAX_ENCODER_ANGLE   = 360;
const int ANGLE_TO_ENC_COUNTS   = ((ENC_CPR * GEAR_RATIO) / (MAX_ENCODER_ANGLE));

uint16_t rightspeed;
uint16_t leftspeed;

const byte DRIVE_ZERO        = 0;       //RPM
const byte ANGLE_DEFAULT     = 90;

const int SWERVE_MIN_ECS     = 0;
const int SWERVE_MAX_ECS     = GEAR_RATIO * ENC_CPR;      //Max Encoder Counts per Second

//Rovecomm
RoveCommEthernet RoveComm;
rovecomm_packet packet;
RoveWatchdog Watchdog;
uint32_t last_update_time;


//All wheels are in order of LF,LR,RF,RR

uint8_t motorButtons[4] = { LF_MOTOR, 
                            LR_MOTOR, 
                            RF_MOTOR, 
                            RR_MOTOR};

int16_t motorSpeeds[4]  = {DRIVE_ZERO, 
                            DRIVE_ZERO, 
                            DRIVE_ZERO, 
                            DRIVE_ZERO}; //LF,LR,RF,RR

uint8_t encoderPins[4]  = { LF_PWM, 
                            LR_PWM, 
                            RF_PWM, 
                            RR_PWM};

uint8_t wheelAngle[4]   = { ANGLE_DEFAULT, 
                            ANGLE_DEFAULT, 
                            ANGLE_DEFAULT, 
                            ANGLE_DEFAULT}; //LF,LR,RF,RR

int16_t turnSpeeds[4] =  { DRIVE_ZERO,
                            DRIVE_ZERO,
                            DRIVE_ZERO,
                            DRIVE_ZERO}; //LF,LR,RF,RR

float motorCurrent[4] = {0,0,0,0};

RoveUsDigiMa3Pwm EncoderLF, EncoderLR, EncoderRF, EncoderRR;
RoveUsDigiMa3Pwm encoders[4]=    {EncoderLF,
                                  EncoderLR,
                                  EncoderRF,
                                  EncoderRR};

RoveUsDigiMa3PwmWireBreaks  WireBreaks;

void EStop();
void swerveDriveInit(uint8_t *wheelAngle);
void pointTurn(uint8_t *wheelAngle);
void moveWheelsToAngle(uint8_t *goalAngle);
void printUARTdata(VescUart UART);


#endif
