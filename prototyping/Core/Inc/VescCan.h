
#ifndef vesccan
#define vesccan
#include "main.h"
#include "stm32h5xx_hal.h"

#define CAN_setcurrent 1
#define CAN_setRPM 3
#define CAN_setPOS 4

void SetRPM(FDCAN_HandleTypeDef *fdcan, uint8_t can_ID, int32_t rpm);
void SetCurrent(FDCAN_HandleTypeDef *fdcan, uint8_t can_ID, float current);
#endif
