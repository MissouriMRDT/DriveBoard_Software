// Left side pwm pins
const byte FL_MOTOR_PWM = 77; // front
const byte ML_MOTOR_PWM = 78; // middle
const byte RL_MOTOR_PWM = 79; // rear
// Right side pwm pins
const byte FR_MOTOR_PWM = 39; // front
const byte MR_MOTOR_PWM = 59; // middle
const byte RR_MOTOR_PWM = 40; // rear
// Left side RX pins
const byte FL_MOTOR_RX = 69; // front
const byte ML_MOTOR_RX = 45; // middle
const byte RL_MOTOR_RX = 54; // rear
// Right side RX pins
const byte FR_MOTOR_RX = 43; // front
const byte MR_MOTOR_RX = 5; // middle
const byte RR_MOTOR_RX = 3; // rear
// Left side TX pins
const byte FL_MOTOR_TX = 70; // front
const byte ML_MOTOR_TX = 46; // middle
const byte RL_MOTOR_TX = 55; // rear
// Right side TX pins
const byte FR_MOTOR_TX = 44; // front
const byte MR_MOTOR_TX = 8; // middle
const byte RR_MOTOR_TX = 4; // rear

const byte MAX_FWD = 127; // maximum forward speed
const byte MAX_REV = -128; // maximum reverse speed
const byte BIT_DELAY = 100; // time to delay between writing bits to pins in send()
const int TEST_LENGTH = 2000; // run each test for 2 seconds

// All arrays ordered: RR, MR, FR, RL, ML, FL
byte rx[] = {RR_MOTOR_RX, MR_MOTOR_RX, FR_MOTOR_RX, RL_MOTOR_RX, ML_MOTOR_RX, FL_MOTOR_RX};
byte tx[] = {RR_MOTOR_TX, MR_MOTOR_TX, FR_MOTOR_TX, RL_MOTOR_TX, ML_MOTOR_TX, FL_MOTOR_TX};
byte pwm[] = {RR_MOTOR_PWM, MR_MOTOR_PWM, FR_MOTOR_PWM, RL_MOTOR_PWM, ML_MOTOR_PWM, FL_MOTOR_PWM};
byte speed[] = {0,0,0,0,0,0}; // holds values for speeds to be sent over Rx line

void setup()
{
  Serial.begin(9600);
  for(int i = 0; i < 6; i++)
  { // set pins to appropriate modes
    pinMode(rx[i], INPUT);
    pinMode(tx[i], OUTPUT);
    pinMode(pwm[i], OUTPUT);
  }
  test();
}

void test() // runs motors at half-speed forward and backwards with pauses in between.
{
  int time;
  Serial.println("   --- Begin Test ---");

  for(int i = 0; i < 6; i++)
    speed[i] = 0;
  Serial.println("> Left - Forward - Half Speed");
  speed[3] = speed[4] = speed[5] = 64; // set left side to half speed forward.
  time = millis();
  while(millis()-time < TEST_LENGTH)
    write_speeds(); // send speeds to motor controllers.

  Serial.println("> Right - Forward - Half Speed");
  speed[3] = speed[4] = speed[5] = 0; // set left side to 0 speed
  speed[1] = speed[2] = speed[3] = 64; // set right side to half speed forward.
  time = millis();
  while(millis()-time < TEST_LENGTH)
    write_speeds(); // send speeds to motor controllers.

  Serial.println("> Left - Backward - Half Speed");
  speed[3] = speed[4] = speed[5] = -64; // set left side to half speed backward
  speed[1] = speed[2] = speed[3] = 0; // set right side to 0 speed.
  time = millis();
  while(millis()-time < TEST_LENGTH)
    write_speeds(); // send speeds to motor controllers.

  Serial.println("> Right - Backward - Half Speed");
  speed[3] = speed[4] = speed[5] = 0; // set left side to 0 speed
  speed[1] = speed[2] = speed[3] = -64; // set right side to half speed backward.
  time = millis();
  while(millis()-time < TEST_LENGTH)
    write_speeds(); // send speeds to motor controllers.  

  // set motor speeds to 0.
  for(int i = 0; i < 6; i++)
    speed[i] = 0;
  write_speeds();  
}
void loop()
{
  // keep speed at 0 in the loop
  for(byte i = 0; i < 6; i++)
    speed[i] = 0;

  write_speeds(); // send the speed bytes to the motor controller over Rx line
}

void write_speeds()
{
  // Code for this function is based on this explanation: https://www.arduino.cc/en/Tutorial/BitMask
  for(byte mask = 00000001; mask > 0; mask <<= 1) // create a bitmask and perform a left-shift until the mask is 00000000.
  {
    for(byte i = 0; i < 6; i++) // cycle through speeds on inner loop to reduce the total delay time.
    {
      if(speed[i] & mask) // set the pin high or low to represent the current bit.
        digitalWrite(tx[i], HIGH); // write a '1'
      else
        digitalWrite(tx[i], LOW); // write a '0'
    }
    delayMicroseconds(BIT_DELAY); // hold the pin high or low for a fixed amount of time.
  }
}

