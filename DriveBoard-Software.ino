// Standard C integers of specific size (such as int16_t)
#include <stdint.h>

// Include Tiva Watchdog Driver Library
#include <driverlib/sysctl.h>
#include <driverlib/watchdog.h>

// Energia libraries used by RoveWare itself
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

// RoveWare 
#include "RoveEthernet.h"
#include "RoveComm.h"

// DRIVE DATE : CONTINOUS ROTATION SERVO BY PWM
//
// https://en.wikipedia.org/wiki/Servo_(radio_control)

// Arduino Library for Rc Pwm Servo (rotate joint angle): min 0 maps to 544 microseconds, max 360 maps to 2400 microseconds
#include <Servo.h>


  // TODO: GBENGA
  
  // CONFIRMED Energia.h conflict #define WTIMER0 6 // this is needed PWMWrite, see TimerPrescaleSet
  
  // period = F_CPU / SYSTICKHZ
  // #define SYSTICKHZ               1000UL
  // #define SYSTICKMS               (1000UL / SYSTICKHZ)
  
  // UTILITIES FOR CLOCK CYCLE ARGS on the 1294 WTF with the hard code 80000000L?
  
  // they use F_CPU everywhere else...
  
  // //#define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )
  // #define clockCyclesPerMicrosecond() ( 80000000L / 1000000L )
  // #define clockCyclesToMicroseconds(a) ( (a) / clockCyclesPerMicrosecond() )
  // #define microsecondsToClockCycles(a) ( (a) * clockCyclesPerMicrosecond() )
  
  // unsigned long micros(void)
  //{
    // return (milliseconds * 1000) + ( ((F_CPU / SYSTICKHZ) - MAP_SysTickValueGet()) / (F_CPU/1000000));
  //}
  
  //unsigned long ticks = (unsigned long)us * (F_CPU/1000000UL);
  
  //HOWEVER, this is a bug in Energia itself on all non 80 Mhz configurations of the 1294 and 432
  
  // https://github.com/energia/Energia/issues/633
  
  // Notice the 430 has : #define clockCyclesPerMicrosecond() ( F_CPU / 1000000L )
  
  // TivaWare DriverLibs  <driverlib/sysctl.h>
  // http://www.ti.com/lit/ug/spmu298a/spmu298a.pdf
  
  // The default internal Oscillator is used at 16 Mhz
  // The value for the System Clock may be set to any integer divider value of the VCO
  // If we want a faster frequency than 16 Mhz we must use an external oscillator or a phase lock loop (PLL) for the VCO
 
  // Phase locked loop system allows lock signal at each various phase to generates frequency in multiple of the applied clock
  // To use can use 80MHz on a board with a 16 MHz external crystal, this 16 Mhz value is multiplied 5 times to get 80 MHz ( SYSDIV2 = 0x02, SYSDIV2LSB = 0, Divisor = 5)
 
  // Texas Instrument Board Sepcs: TivaWare SysClock Board Periphs :
  
  // 129x : PLL and VCO 320 Mhz or 480 Mhz only -> max sysclock 120 Mhz
  
  // 123x : PLL and VCO 200 Mhz or 400 Mhz only -> max sysclock 80 Mhz
  
  // 432x : DCO is power-efficient tunable internal oscillator that generates up to 48 MHz with high-precision mode for external precision resistor, 
  //      : Module Oscillator (MODOSC) MODOSC is factory calibrated frequency of 25 MHz to supply a 'clock on request' to modules like the ADC (in 1-Msps conversion mode).
  //      : System Oscillator (SYSOSC) lower-frequency version of MODOSC factory calibrated to a frequency of 5MHz. to drive ADC sampling clock in the 200-ksps conversion mode
  
  // VCO PLL http://www.radio-electronics.com/info/rf-technology-design/pll-synthesizers/phase-locked-loop-tutorial.php
  // http://arm-tutorials.com/2016/02/13/pll-and-clock-settings-in-tm4c123gh6pm/
  

  // As an example, 80MHz can be got from both VCO of 320 and 480, 120Mhz can only be got from VCO of 480
 
  //Native
  
  // 1294:
  // #define MY_SKETCH_SYSCLOCK_HZ 120000000
  // uint32_t system_clock_freq_in_hz = SysCtlClockFreqSet(SYSCTL_OSC_INT | SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480, MY_SKETCH_SYSCLOCK_HZ);
  // SysCtlClockFreqSet returns The actual configured system clock frequency in Hz or zero if the value could not be changed due to a parameter error or PLL lock failure
  
  
  // 123G:
  // #define MY_SKETCH_SYSCLOCK_HZ 120000000
  // uint32_t system_clock_freq_in_hz = SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
  
  // SysCtlClockFreqSet returns The actual configured system clock frequency in Hz or zero if the value could not be changed due to a parameter error or PLL lock failure
  
  
  
  //Wiring: 
  // 123G default: Run at system clock at 80MHz
  //
  // MAP_SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
  
  
  
  // 1294 default: Run at system clock at 120MHz
  // MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ|SYSCTL_OSC_MAIN|SYSCTL_USE_PLL|SYSCTL_CFG_VCO_480), F_CPU);
    
  // END GBENGA
  



void timerInit()
{
#ifdef TARGET_IS_BLIZZARD_RB1
    //
 
#else
    //
    //  Run at system clock at 120MHz
    //
    MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ|SYSCTL_OSC_MAIN|SYSCTL_USE_PLL|SYSCTL_CFG_VCO_480), F_CPU);
#endif

    //
    //  SysTick is used for delay() and delayMicroseconds()
    //

    MAP_SysTickPeriodSet(F_CPU / SYSTICKHZ);
    MAP_SysTickEnable();
    MAP_IntPrioritySet(FAULT_SYSTICK, SYSTICK_INT_PRIORITY);
    MAP_SysTickIntEnable();
    MAP_IntMasterEnable();

    // PIOSC is used during Deep Sleep mode for wakeup
    MAP_SysCtlPIOSCCalibrate(SYSCTL_PIOSC_CAL_FACT);  // Factory-supplied calibration used
}



  // TODO: JOHN MARUSKA
  // 123G : Calculate the actual system clock.
  //
  // ui32SysClock = _SysCtlFrequencyGet(ui32Osc) / ui32SysDiv;
  //end JOHN
  

  // Enable Watchdog Timer 1; supposedly timer 0 has a conflict in Energia, unconfirmed
  SysCtlPeripheralEnable(SYSCTL_PERIPH_WDOG1);
// Servo goes a position between 0 and 360
int position = 0;

Servo left_front_motor;
Servo left_middle_motor;
Servo left_back_motor;

Servo right_front_motor;
Servo right_middle_motor;
Servo right_back_motor;

// Rc Pwm protocol for (spin wheel speed) bis not by position : full reverse maps to 1000 microseconds, full forward maps to 2000 microseconds 
const int RC_PWM_MIN_SPEED      =   1000;
const int RC_PWM_MAX_SPEED      =   2000;

// RED sends us a data_id and a motor_speed
uint16_t data_id = 0;

const uint16_t DRIVE_LEFT_MOTORS_BY_IP = 100;
const uint16_t DRIVE_RIGHT_MOTORS_BY_IP = 101;

// Not used: We already know the size of motor_speed (int16_t), but Rovecomm does supports variable command sizes, so we always pass in data_size
size_t data_size = 0; 

// Red sends xbox controller input to us
const int16_t RED_MIN_SPEED     =  -1000;
const int16_t RED_MAX_SPEED     =   1000;

const int position_zero = map(0, RED_MIN_SPEED, RED_MAX_SPEED, 0,  180);

int16_t motors_speed = 0;

uint32_t ROVECOMMWATCHDOG = WATCHDOG1_BASE;
#define WATCHDOG_LENGTH 30000000 //Randomly chosen number, takes roughly 1.5 seconds

void stopDriving() { 
  //Serial.println("Watchdog Triggered");

  left_front_motor.write(position_zero);
  left_middle_motor.write(position_zero);
  left_back_motor.write(position_zero);
  right_front_motor.write(position_zero);
  right_middle_motor.write(position_zero);  
  right_back_motor.write(position_zero);

  WatchdogIntClear(ROVECOMMWATCHDOG);
}//end function

void setup()
{
  //Serial.begin(9600);  
  roveComm_Begin(192, 168, 1, 130); 

  //Tiva Pin: PG0 = 37
  left_front_motor.attach(37, RC_PWM_MIN_SPEED, RC_PWM_MAX_SPEED);
  //Tiva Pin: PF3 = 38
  left_middle_motor.attach(38, RC_PWM_MIN_SPEED, RC_PWM_MAX_SPEED);
  //PF2
  left_back_motor.attach(39, RC_PWM_MIN_SPEED, RC_PWM_MAX_SPEED);

  //PG1
  right_front_motor.attach(80, RC_PWM_MIN_SPEED, RC_PWM_MAX_SPEED);  
  //PK4
  right_middle_motor.attach(79, RC_PWM_MIN_SPEED, RC_PWM_MAX_SPEED);
  //PK_5
  right_back_motor.attach(78, RC_PWM_MIN_SPEED, RC_PWM_MAX_SPEED);

  WatchdogUnlock(ROVECOMMWATCHDOG);
  WatchdogReloadSet(ROVECOMMWATCHDOG, WATCHDOG_LENGTH);
  WatchdogIntRegister(ROVECOMMWATCHDOG, stopDriving);
  WatchdogIntEnable(ROVECOMMWATCHDOG);
  WatchdogResetDisable(ROVECOMMWATCHDOG);
  WatchdogEnable(ROVECOMMWATCHDOG);

}//end setup

void loop()
{
  //If there is no message data_id gets set to zero
  roveComm_GetMsg(&data_id, &data_size, &motors_speed);

  switch (data_id) 
  {   
    //Don't do anything for data_id zero 
  case 0:
    break;

  case DRIVE_LEFT_MOTORS_BY_IP:   

    //This is the tricky part. We are hacking a 0 to 360 angle control servo library to write out continous rotation speed by mapping one to the other
    position = map((int)motors_speed, RED_MIN_SPEED, RED_MAX_SPEED, 0,  180); 

    left_front_motor.write(position);
    left_middle_motor.write(position);
    left_back_motor.write(position); 
    //Serial.println(motors_speed);
    //Serial.println(pos);
    break;

  case DRIVE_RIGHT_MOTORS_BY_IP: 

    position = map((int)motors_speed, RED_MIN_SPEED, RED_MAX_SPEED, 0,  180); 

    right_front_motor.write(position);
    right_middle_motor.write(position);  
    right_back_motor.write(position); 
    //Serial.println(motors_speed);
    //Serial.println(pos); 
    break;

  default:
    //Serial.print("RoveDrive unrecognized data_id :");
    //Serial.println(data_id);
    break;

  }//endswitch

  if (data_id != 0) {
    WatchdogIntClear(ROVECOMMWATCHDOG);
  }//end if

}//end loop

