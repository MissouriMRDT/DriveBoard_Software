#include "DriveBoardSoftware.h"

void setup() {

    // Start RoveComm
    RoveComm.begin(RC_DRIVEBOARD_FIRSTOCTET, RC_DRIVEBOARD_SECONDOCTET, RC_DRIVEBOARD_THIRDOCTET, RC_DRIVEBOARD_FOURTHOCTET, &TCPServer);
    delay(100);
    
    // Initialize debug serial port
    Serial.begin(9600);
    Serial.println("Serial init");
    Serial.println("RoveComm Init");

    // Set override buttons to input
    for(int i = 0; i < 6; i++) {
        pinMode(motorButtons[i], INPUT);
    }

    lastRampTime = millis();

    watchdog.begin(EStop, WATCHDOG_TIME);
    

    // Initialize VESC serial ports
    FL_SERIAL.begin(115200);
    while(!(FL_SERIAL));
    
    ML_SERIAL.begin(115200);
    while(!(ML_SERIAL));
    
    BL_SERIAL.begin(115200);
    while(!(BL_SERIAL));
    
    FR_SERIAL.begin(115200);
    while(!(FR_SERIAL));
    
    MR_SERIAL.begin(115200);
    while(!(MR_SERIAL));
    
    BR_SERIAL.begin(115200);
    while(!(BR_SERIAL));

    FL_UART.setSerialPort(&FL_SERIAL);
    ML_UART.setSerialPort(&ML_SERIAL);
    BL_UART.setSerialPort(&BL_SERIAL);
    FR_UART.setSerialPort(&FR_SERIAL);
    MR_UART.setSerialPort(&MR_SERIAL);
    BR_UART.setSerialPort(&BR_SERIAL);
    
    telemetry.begin(Telemetry, TELEMETRY_UPDATE);

}

void loop() 
{
    // Read incoming packet
    packet = RoveComm.read();
    
    switch(packet.data_id) 
    {
        case RC_DRIVEBOARD_DRIVELEFTRIGHT_DATA_ID:
        {
            // Read packet and cast to correct type
            int16_t* lrspeeds;
            lrspeeds = (int16_t*)packet.data;

            // Map speed values from +-1000 to max and min rpm
            // TODO implement min drive rmp
            int16_t leftSpeed = map(lrspeeds[0], -1000, 1000, -DRIVE_MAX_RPM, DRIVE_MAX_RPM);
            int16_t rightSpeed = map(lrspeeds[1], -1000, 1000, -DRIVE_MAX_RPM, DRIVE_MAX_RPM);

            // Send RPM values to VESCs (First 3 are left, next 3 are right)
            for(int i = 0; i < 6; i++) 
                motorTargets[i] = (i < 3) ? leftSpeed : rightSpeed;

            watchdog.begin(EStop, WATCHDOG_TIME);

            FL_UART.setRPM((float)leftSpeed);
            ML_UART.setRPM((float)leftSpeed);
            BL_UART.setRPM((float)leftSpeed);
            FR_UART.setRPM((float)rightSpeed);
            MR_UART.setRPM((float)rightSpeed);
            BR_UART.setRPM((float)rightSpeed);
            break;
        }
        case RC_DRIVEBOARD_DRIVEINDIVIDUAL_DATA_ID:
        {
            // Read packet and cast to correct type
            int16_t* speeds;
            speeds = (int16_t*)packet.data;

            // Map speed values and send to VESCs
            for(int i = 0; i < 6; i++) 
                motorTargets[i] = map(speeds[i], -1000, 1000, -DRIVE_MAX_RPM, DRIVE_MAX_RPM);

            watchdog.begin(EStop, WATCHDOG_TIME);
            break;
        }
    }
    

    // Override speeds if button is pressed
    maxRamp = (millis() - lastRampTime) * DRIVE_MAX_RAMP;
    for(int i = 0; i < 6; i++)
    {
        if(digitalRead(motorButtons[i]))
        {
          //Invert motor output if switch is toggled
          if(digitalRead(DIR_SWITCH))
          {
            motorTargets[i] = -BUTTON_OVERIDE_SPEED;
          } 
         else 
          {
            motorTargets[i] = BUTTON_OVERIDE_SPEED;
          }
        }

        if((motorTargets[i] > motorSpeeds[i]) && ((motorTargets[i] - motorSpeeds[i]) > maxRamp))
        {
            motorSpeeds[i] += maxRamp;
            Serial.println("Ramping Up");
        }
        else if((motorTargets[i] < motorSpeeds[i]) && ((motorTargets[i] - motorSpeeds[i]) < -maxRamp))
        {
            motorSpeeds[i] -= maxRamp;
            Serial.println("Ramping Down");
        }
        else 
            motorSpeeds[i] = motorTargets[i];
    }
    
    
    lastRampTime = millis();

    FL_UART.setRPM((float)motorSpeeds[0]);
    ML_UART.setRPM((float)motorSpeeds[1]);
    BL_UART.setRPM((float)motorSpeeds[2]);
    FR_UART.setRPM((float)motorSpeeds[3]);
    MR_UART.setRPM((float)motorSpeeds[4]);
    BR_UART.setRPM((float)motorSpeeds[5]);
}

void EStop() 
{    
    if(!watchdogOverride) 
    {
        for(int i = 0; i < 6; i++) 
            motorTargets[i] = 0;

        watchdog.begin(EStop, WATCHDOG_TIME);
    }   
}

void Telemetry()
{

    //Converts Motor Current to a value from {-DRIVE_MAX_RPM, DRIVE_MAX_RPM} -> {-1000, 1000}
    if(FL_UART.getVescValues()) {
        motorCurrent[0] = (float)map(FL_UART.data.rpm, -DRIVE_MAX_RPM, DRIVE_MAX_RPM, -1000, 1000);
    } else {
        motorCurrent[0] = 0;
    }


    if(ML_UART.getVescValues()) {
        motorCurrent[1] = (float)map(ML_UART.data.rpm, -DRIVE_MAX_RPM, DRIVE_MAX_RPM, -1000, 1000);
    } else {
        motorCurrent[1] = 0;
    }


    if(BL_UART.getVescValues()) {
        motorCurrent[2] = (float)map(BL_UART.data.rpm, -DRIVE_MAX_RPM, DRIVE_MAX_RPM, -1000, 1000);
    } else {
        motorCurrent[2] = 0;
    }

    
    if(FR_UART.getVescValues()) {
        motorCurrent[3] = (float)map(FR_UART.data.rpm, -DRIVE_MAX_RPM, DRIVE_MAX_RPM, -1000, 1000);
    } else {
        motorCurrent[3] = 0;
    }


    if(MR_UART.getVescValues()) {
        motorCurrent[4] = (float)map(MR_UART.data.rpm, -DRIVE_MAX_RPM, DRIVE_MAX_RPM, -1000, 1000);
    } else {
        motorCurrent[4] = 0;
    }


    if(BR_UART.getVescValues()) {
        motorCurrent[5] = (float)map(BR_UART.data.rpm, -DRIVE_MAX_RPM, DRIVE_MAX_RPM, -1000, 1000);
    } else {
        motorCurrent[5] = 0;
    }

    RoveComm.write(RC_DRIVEBOARD_DRIVESPEEDS_DATA_ID, RC_DRIVEBOARD_DRIVESPEEDS_DATA_COUNT, motorCurrent);
}
