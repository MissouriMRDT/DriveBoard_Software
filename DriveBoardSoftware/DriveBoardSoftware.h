
#ifndef _DriveBoard
#define _DriveBoard

#include "RoveComm.h"
#include <VescUart.h>
#include "RoveUsDigiMa3Pwm.h"
#include "RovesODrive.h"
#include "RoveWatchdog.h"

//Motor Overide Buttons
#define FR_MOTOR                PL_5
#define FL_MOTOR                PL_4
#define RR_MOTOR                PL_1
#define RL_MOTOR                PL_0
#define DIR_SWITCH              PP_2
#define BUTTON_OVERIDE_SPEED    5000

//Encoder PWM input pins
#define FL_PWM              PF_1
#define FR_PWM              PF_2
#define RL_PWM              PF_3
#define RR_PWM              PG_0

//SwerveDrive
#define WHEEL_TURN_SPEED                    2000 //RPM
#define DEGREE_ALLOWABLE_INIT_DIFFERENCE    1    //Initialize wheel turn when within this degree difference
#define DEGREE_OFFSET_TOLERANCE             1    //stops rover to readjust wheels is above tolerance 

//Motor Speed Controls
#define FL_SERIAL               Serial2
#define FR_SERIAL               Serial3
#define RL_SERIAL               Serial4
#define RR_SERIAL               Serial6

VescUart FL_UART;
VescUart FR_UART;
VescUart RL_UART;
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
const int GEAR_RATIO            = 600;
const float MAX_ENCODER_ANGLE   = 360;
const int ANGLE_TO_ENC_COUNTS   = ((ENC_CPR * GEAR_RATIO) / (MAX_ENCODER_ANGLE));

uint16_t rightspeed;
uint16_t leftspeed;

const byte DRIVE_ZERO        = 0;       //RPM
const byte ANGLE_DEFAULT     = 90;

const int SWERVE_MIN_RPM     = 0;
const int SWERVE_MAX_RMP     = 20000;

//Rovecomm
RoveCommEthernet RoveComm;
rovecomm_packet packet;
RoveWatchdog Watchdog;


//All wheels are in order of FL,RL,FR,RR

uint8_t motorButtons[4] = { FL_MOTOR, 
                            RL_MOTOR, 
                            FR_MOTOR, 
                            RR_MOTOR};

uint16_t motorSpeeds[4]  = {DRIVE_ZERO, 
                            DRIVE_ZERO, 
                            DRIVE_ZERO, 
                            DRIVE_ZERO}; //FL, RL, FR, RR

uint8_t encoderPins[4]  = { FL_PWM, 
                            RL_PWM, 
                            FR_PWM, 
                            RR_PWM};

uint8_t wheelAngle[4]   = { ANGLE_DEFAULT, 
                            ANGLE_DEFAULT, 
                            ANGLE_DEFAULT, 
                            ANGLE_DEFAULT}; //FL, RL, FR, RR

RoveUsDigiMa3Pwm EncoderFL, EncoderRL, EncoderFR, EncoderRR;
RoveUsDigiMa3Pwm encoders[4]=    {EncoderFL,
                                  EncoderRL,
                                  EncoderFR,
                                  EncoderRR};

RoveUsDigiMa3PwmWireBreaks  WireBreaks;

void EStop();
void swerveDriveInit(uint8_t *wheelAngle);
void pointTurn(uint8_t *wheelAngle);
void moveWheelsToAngle(uint8_t *goalAngle);
void printUARTdata(VescUart UART);


#endif
