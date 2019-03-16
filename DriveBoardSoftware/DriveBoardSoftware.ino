/*
 * DriveBoard Software Rev 2 2013
 * Used with ODrive Motor Controllers
 * Writes Serial to 3 motor drives
 * 
 * Andrew Van Horn
 */

#include "RoveComm.h"
#include "RovesODrive.h"
#include "DriveBoardSoftware.h"

RovesODrive Drive[] = {&FRONTDRIVE_SERIAL, &MIDDLEDRIVE_SERIAL, &REARDRIVE_SERIAL);

void setup()
{
    pinMode(DIRECTION_SWITCH_PIN, INPUT);
    pinMode(LF_BUTTON_PIN       , INPUT);
    pinMode(LM_BUTTON_PIN       , INPUT);
    pinMode(LR_BUTTON_PIN       , INPUT);
    pinMode(RF_BUTTON_PIN       , INPUT);
    pinMode(RM_BUTTON_PIN       , INPUT);
    pinMode(RR_BUTTON_PIN       , INPUT);
                                  
    pinMode(WATCHDOG_IND_PIN    , OUTPUT);
    pinMode(LSPEED_IND_PIN      , OUTPUT);
    pinMode(RSPEED_IND_PIN      , OUTPUT);
}

void loop()
{

}