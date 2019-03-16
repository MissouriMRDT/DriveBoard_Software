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

RovesODrive Drive[] = {&FRONTDRIVE_SERIAL, &MIDDRIVE_SERIAL, &REARDRIVE_SERIAL};

void chackButtons()
{

}

void setup()
{
    pinMode(DIRECTION_SWITCH_PIN, INPUT);

    for(int i = 0; i<NUMMOTORS; i++)
    {
      pinMode(motor[i].button_pin, INPUT);
    }
                                  
    pinMode(WATCHDOG_IND_PIN    , OUTPUT);
    pinMode(LSPEED_IND_PIN      , OUTPUT);
    pinMode(RSPEED_IND_PIN      , OUTPUT);

    for(int i = 0; i<NUMDRIVES; i++)
    {
      Drive[i].begin();
    }
}

void loop()
{

}