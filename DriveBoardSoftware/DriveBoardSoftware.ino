#include "DriveBoardSoftware.h"

void setup() {

    // Initialize debug serial port
    Serial.begin(9600);

    Serial.println("Serial init");

    // Initialize VESC serial ports
    FL_SERIAL.begin(115200);
    ML_SERIAL.begin(115200);
    BL_SERIAL.begin(115200);
    FR_SERIAL.begin(115200);
    MR_SERIAL.begin(115200);
    BR_SERIAL.begin(115200);

    while(!(FL_SERIAL && FR_SERIAL && ML_SERIAL && MR_SERIAL && BL_SERIAL && BR_SERIAL));

    FL_UART.setSerialPort(&FL_SERIAL);
    ML_UART.setSerialPort(&ML_SERIAL);
    BL_UART.setSerialPort(&BL_SERIAL);
    FR_UART.setSerialPort(&FR_SERIAL);
    MR_UART.setSerialPort(&MR_SERIAL);
    BR_UART.setSerialPort(&BR_SERIAL);

    // Start RoveComm
    RoveComm.begin(RC_DRIVEBOARD_FOURTHOCTET, &TCPServer, RC_ROVECOMM_DRIVEBOARD_MAC);
    delay(100);

    Serial.println("RoveComm Init");

    // Set override buttons to input
    for(int i = 0; i < 6; i++) {
        pinMode(motorButtons[i], INPUT);
    }

    watchdog.begin(EStop, 1500000);
    telemetry.begin(Telemetry, 150000);
}

void loop() {

    // Read incoming packet
    packet = RoveComm.read();
    
    switch(packet.data_id) {
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
            for(int i = 0; i < 6; i++) motorTargets[i] = (i < 3) ? leftSpeed : rightSpeed;

            //Timer1.restart();
            watchdog.begin(EStop, 1500000);

            FL_UART.setRPM((float)leftSpeed);
            ML_UART.setRPM((float)leftSpeed);
            BL_UART.setRPM((float)leftSpeed);
            FR_UART.setRPM((float)rightSpeed);
            MR_UART.setRPM((float)rightSpeed);
            BR_UART.setRPM((float)rightSpeed);

            Serial.println("Drive Left Right " + (String)leftSpeed + " " + (String)rightSpeed);

            break;
        }
        case RC_DRIVEBOARD_DRIVEINDIVIDUAL_DATA_ID:
        {
            // Read packet and cast to correct type
            int16_t* speeds;
            speeds = (int16_t*)packet.data;

            // Map speed values and send to VESCs
            for(int i = 0; i < 6; i++) motorTargets[i] = map(speeds[i], -1000, 1000, -DRIVE_MAX_RPM, DRIVE_MAX_RPM);

            //Timer1.restart();
            watchdog.begin(EStop, 1500000);


            Serial.println("Drive Individual");

            break;
        }
        case RC_DRIVEBOARD_WATCHDOGOVERRIDE_DATA_ID:
        {
            //Read packet and cast to correct type
            if(((uint8_t*)packet.data)[0] == (uint8_t)1) //Timer1.stop();
            if(((uint8_t*)packet.data)[0] == (uint8_t)0) //Timer1.resume();

            break;
        }
    }

    // Override speeds if button is pressed
    maxRamp = (millis() - lastRampTime) * DRIVE_MAX_RAMP;
    for(int i = 0; i < 6; i++)
    {
        if(digitalRead(motorButtons[i])) motorTargets[i] = BUTTON_OVERIDE_SPEED;
        if((motorTargets[i] > motorSpeeds[i]) && ((motorTargets[i] - motorSpeeds[i]) > maxRamp)){
            motorSpeeds[i] += maxRamp;
            Serial.println("Ramping Up");
        }else if((motorTargets[i] < motorSpeeds[i]) && ((motorTargets[i] - motorSpeeds[i]) < -maxRamp)){
            motorSpeeds[i] -= maxRamp;
            Serial.println("Ramping Down");
        }else motorSpeeds[i] = motorTargets[i];
    }
    lastRampTime = millis();

    Serial.println("BR Motor Speed = " + (String)motorSpeeds[5]);

    FL_UART.setRPM((float)motorSpeeds[0]);
    ML_UART.setRPM((float)motorSpeeds[1]);
    BL_UART.setRPM((float)motorSpeeds[2]);
    FR_UART.setRPM((float)motorSpeeds[3]);
    MR_UART.setRPM((float)motorSpeeds[4]);
    BR_UART.setRPM((float)motorSpeeds[5]);

}

void EStop() {
    
    if(!watchdogOverride) {

        for(int i = 0; i < 6; i++) motorTargets[i] = 0;

        //Timer1.restart();
        watchdog.begin(EStop, 1500000);

    }
    
}

void Telemetry()
{
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

        RoveComm.write(RC_DRIVEBOARD_DRIVESPEEDS_DATA_ID, 6, motorCurrent);

        telemetry.begin(Telemetry, 150000);
}