// Standard C integers of specific size (such as int16_t)
#include <stdint.h>

// Left side pwm pins
const uint16_t FL_MOTOR_PWM = 77; // front
const uint16_t ML_MOTOR_PWM = 78; // middle
const uint16_t RL_MOTOR_PWM = 79; // rear
// Right side pwm pins
const uint16_t FR_MOTOR_PWM = 39; // front
const uint16_t MR_MOTOR_PWM = 59; // middle
const uint16_t RR_MOTOR_PWM = 40; // rear
// Left side RX pins
const uint16_t FL_MOTOR_RX = 69; // front
const uint16_t ML_MOTOR_RX = 45; // middle
const uint16_t RL_MOTOR_RX = 54; // rear
// Right side RX pins
const uint16_t FR_MOTOR_RX = 43; // front
const uint16_t MR_MOTOR_RX = 5; // middle
const uint16_t RR_MOTOR_RX = 3; // rear
// Left side TX pins
const uint16_t FL_MOTOR_TX = 70; // front
const uint16_t ML_MOTOR_TX = 46; // middle
const uint16_t RL_MOTOR_TX = 55; // rear
// Right side TX pins
const uint16_t FR_MOTOR_TX = 44; // front
const uint16_t MR_MOTOR_TX = 8; // middle
const uint16_t RR_MOTOR_TX = 4; // rear

void setup()
{
  Serial.begin(9600);
  // left side test
  Serial.println("Driving left motors at half speed...");
  drive_side(true, 127);
  delay(1000);
  drive_side(true, 0);
  // right side test
  Serial.println("Driving right motors at half speed...");
  drive_side(false, 127);
  delay(1000);
  drive_side(false, 0);
}//end setup

void loop(){}

void drive_side(bool side, int power) // side is true for left, false for right
{
  // limit the pwm signal
  if(power < 0) power = 0;
  else if (power > 255) power = 255;
  if(side)
  {
    analogWrite(FL_MOTOR_PWM, power);
    analogWrite(ML_MOTOR_PWM, power);
    analogWrite(RL_MOTOR_PWM, power);
  }
  else
  {
    analogWrite(RL_MOTOR_PWM, power);
    analogWrite(RL_MOTOR_PWM, power);
    analogWrite(RL_MOTOR_PWM, power);
  }
}
