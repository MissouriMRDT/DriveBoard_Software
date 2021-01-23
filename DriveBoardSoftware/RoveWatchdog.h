#ifndef RoveWatchdog_h
#define RoveWatchdog_h

#include "stdint.h"

#define WATCHDOG_0    0x0
#define WATCHDOG_1    0x1
    
class RoveWatchdog
{
private:
   uint32_t _WATCHDOG_BASE;
public:  
  void begin(void(*watchdogFunction)(void), unsigned int timeout_millis, unsigned int WatchdogTimerModule=WATCHDOG_1); 
  //re-named due to multiple clear watchdog functions and library dependencies
  void clearWatchdog();  
};

#endif // RoveWatchdog_h
