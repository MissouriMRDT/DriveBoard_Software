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

}

void loop()
{

}