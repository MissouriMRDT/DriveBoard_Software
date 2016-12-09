//// Left side pwm pins
//const byte FL_MOTOR_PWM = 77; // front
//const byte ML_MOTOR_PWM = 78; // middle
//const byte RL_MOTOR_PWM = 79; // rear
//// Right side pwm pins
//const byte FR_MOTOR_PWM = 39; // front
//const byte MR_MOTOR_PWM = 59; // middle
//const byte RR_MOTOR_PWM = 40; // rear
//// Left side RX pins
//const byte FL_MOTOR_RX = 69; // front
//const byte ML_MOTOR_RX = 45; // middle
//const byte RL_MOTOR_RX = 54; // rear
//// Right side RX pins
//const byte FR_MOTOR_RX = 43; // front
//const byte MR_MOTOR_RX = 5; // middle
//const byte RR_MOTOR_RX = 3; // rear
//// Left side TX pins
//const byte FL_MOTOR_TX = 70; // front
//const byte ML_MOTOR_TX = 46; // middle
//const byte RL_MOTOR_TX = 55; // rear
//// Right side TX pins
//const byte FR_MOTOR_TX = 44; // front
//const byte MR_MOTOR_TX = 8; // middle
//const byte RR_MOTOR_TX = 4; // rear
//
//const byte MAX_FWD = 127; // maximum forward speed
//const byte MAX_REV = -128; // maximum reverse speed
//const byte BIT_DELAY = 100; // time to delay between writing bits to pins in send()
//const int TEST_LENGTH = 2000; // run each test for 2 seconds
//
//// All arrays ordered: RR, MR, FR, RL, ML, FL
//byte rx[] = {RR_MOTOR_RX, MR_MOTOR_RX, FR_MOTOR_RX, RL_MOTOR_RX, ML_MOTOR_RX, FL_MOTOR_RX};
//byte tx[] = {RR_MOTOR_TX, MR_MOTOR_TX, FR_MOTOR_TX, RL_MOTOR_TX, ML_MOTOR_TX, FL_MOTOR_TX};
//byte pwm[] = {RR_MOTOR_PWM, MR_MOTOR_PWM, FR_MOTOR_PWM, RL_MOTOR_PWM, ML_MOTOR_PWM, FL_MOTOR_PWM};
//byte speed[] = {0,0,0,0,0,0}; // holds values for speeds to be sent over Rx line
//

const byte HOLD_TIME = 50; // time to hold the speed in ms.

// speed constants
const byte MAX_FORWARD = 255;
const byte ZERO_SPEED = 127;
const byte MAX_REVERSE = 0;

byte speed = 100;
short last_time = 0;

void setup()
{
  // open output serial channel
  Serial.begin(9600);
  // open motor controller serial channels
  Serial1.begin(19200);
  Serial2.begin(19200);
  Serial3.begin(19200);
  Serial4.begin(19200);
  Serial5.begin(19200);
  Serial6.begin(19200);
  Serial7.begin(19200);

  delay(2000); // wait for a bit before starting test routine

  Serial.println(">  Beginning forward cycle...");
  forward_cycle();
  Serial.println(">  Beginning reverse cycle...");
  reverse_cycle();

  speed = ZERO_SPEED;
}
void loop() 
{  
  Serial1.write(speed);
  Serial2.write(speed);
  Serial3.write(speed);
  Serial4.write(speed);
  Serial5.write(speed);
  Serial6.write(speed);
  Serial7.write(speed);  
}

void forward_cycle()
{
  short accel = 1;
  speed = ZERO_SPEED+1; 
  last_time = 0;
  
  while(speed >= ZERO_SPEED)
  {
    if(millis() - last_time >= HOLD_TIME) // keep sending speeds for HOLD_TIME milliseconds
    {
      speed += accel; // add acceleration to the speed.

      if(speed >= MAX_FORWARD) // if speed hits max...
        accel *= -1; // negate acceleration.
        
      last_time = millis(); // update time
      Serial.print(speed + " "); // output the current speed      
    }
    // write speeds
    Serial1.write(speed);
    Serial2.write(speed);
    Serial3.write(speed);
    Serial4.write(speed);
    Serial5.write(speed);
    Serial6.write(speed);
    Serial7.write(speed);
  }
  Serial.println();
}

void reverse_cycle()
{
  short accel = 1;
  speed = ZERO_SPEED - 1;
  last_time = 0;
  
  while(speed <= ZERO_SPEED)
  {
    if(millis() - last_time >= HOLD_TIME) // keep sending speed fot HOLD_TIME milliseconds
    {
      speed -= accel; // add acceleration to speed (negative accel increases speed since it is increasing in reverse direction.

      if(speed <= MAX_REVERSE) // if speed hits max...
        accel *= -1; // negate acceleration.

      last_time = millis(); // update time
      Serial.print(speed + " "); // output the current speed          
    }
    // write speeds
    Serial1.write(speed);
    Serial2.write(speed);
    Serial3.write(speed);
    Serial4.write(speed);
    Serial5.write(speed);
    Serial6.write(speed);
    Serial7.write(speed);
  }
  Serial.println();
}







