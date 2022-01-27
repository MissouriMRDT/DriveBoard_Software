#include "DriveBoardSoftware.h"

void setup() {

    // Initialize debug serial port
    Serial.begin(9600);

    Serial.println("Serial init");

    // Initialize VESC serial ports
    for(int i = 0; i < 6; i++) motorSerial[i].begin(115200);

    //while(!(FL_SERIAL && FR_SERIAL && ML_SERIAL && MR_SERIAL && BL_SERIAL && BR_SERIAL));

    for(int i = 0; i < 6; i++) motorUART[i].setSerialPort(&motorSerial[i]);

    // Start RoveComm
    RoveComm.begin(RC_DRIVEBOARD_FOURTHOCTET, &TCPServer);
    delay(100);

    Serial.println("RoveComm Init");

    // Start Watchdog, stop motors after 250ms
    Watchdog.start(250);

    Serial.println("Watchdog Start");

    // Set override buttons to input
    for(int i = 0; i < 6; i++) {
        pinMode(motorButtons[i], INPUT);
    }
}

void loop() {

    Serial.println("Loop Start");

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
            for(int i = 0; i < 6; i++) motorSpeeds[i] = (i < 3) ? leftSpeed : rightSpeed;

            Watchdog.clear();

            break;
        }
        case RC_DRIVEBOARD_DRIVEINDIVIDUAL_DATA_ID:
        {
            // Read packet and cast to correct type
            int16_t* speeds;
            speeds = (int16_t*)packet.data;

            // Map speed values and send to VESCs
            for(int i = 0; i < 6; i++) motorSpeeds[i] = map(speeds[i], -1000, 1000, -DRIVE_MAX_RPM, DRIVE_MAX_RPM);

            Watchdog.clear();

            break;
        }
        case RC_DRIVEBOARD_WATCHDOGOVERRIDE_DATA_ID:
        {
            //Read packet and cast to correct type
            watchdogOverride = (((uint8_t*)packet.data)[0] == (uint8_t)1);

            break;
        }
    }

    // Override speeds if button is pressed
    for(int i = 0; i < 6; i++) if(digitalRead(motorButtons[i])) motorSpeeds[i] = BUTTON_OVERIDE_SPEED;

    for(int i = 0; i < 6; i++) motorUART[i].setRPM((float)motorSpeeds[i]);

    if(millis() - lastUpdateTime >= ROVECOMM_UPDATE_RATE) {

        for(int i = 0; i < 6; i++) {

            if(motorUART[i].getVescValues()) {

                motorCurrent[i] = (float)map(motorUART[i].data.rpm, -DRIVE_MAX_RPM, DRIVE_MAX_RPM, -1000, 1000);

            } else {

                motorCurrent[i] = 0;

            }

        }

        RoveComm.writeReliable(RC_DRIVEBOARD_DRIVESPEEDS_DATA_ID, 6, motorCurrent);

        lastUpdateTime = millis();
    
    }

}

void EStop() {
    
    if(!watchdogOverride) {

        for(int i = 0; i < 6; i++) motorUART[i].setRPM((float)0);

        Watchdog.clear();
    }
    
}