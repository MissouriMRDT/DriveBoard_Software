#include "main.h"
#include "stm32h5xx_hal.h"
#include "VescCan.h"



void SetRPM(FDCAN_HandleTypeDef *fdcan, uint8_t can_ID, int32_t rpm){
	FDCAN_TxHeaderTypeDef hdr;
	uint8_t TXdata[4];
	hdr.Identifier = ((uint32_t)CAN_setRPM <<8) | can_ID;
	hdr.IdType = FDCAN_EXTENDED_ID;
	hdr.TxFrameType = FDCAN_DATA_FRAME;
	hdr.DataLength = FDCAN_DLC_BYTES_4;
	hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	hdr.BitRateSwitch = FDCAN_BRS_OFF;
	hdr.FDFormat = FDCAN_FD_CAN;
	hdr.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	hdr.MessageMarker = 0;

	TXdata[0] = (uint8_t)(rpm >> 24);
	TXdata[1] = (uint8_t)(rpm >> 16);
	TXdata[2] = (uint8_t)(rpm >> 8);
	TXdata[3] = (uint8_t)(rpm >> 0);

	HAL_FDCAN_AddMessageToTxFifoQ(fdcan, &hdr, TXdata);

}


void SetCurrent(FDCAN_HandleTypeDef *fdcan, uint8_t can_ID, float current){
	FDCAN_TxHeaderTypeDef hdr;
	int32_t current_scaled = (int32_t)(current * 1000.0f);
	uint8_t TXdata[4];
	hdr.Identifier = ((uint32_t)CAN_setcurrent <<8) | can_ID;
	hdr.IdType = FDCAN_EXTENDED_ID;
	hdr.TxFrameType = FDCAN_DATA_FRAME;
	hdr.DataLength = FDCAN_DLC_BYTES_4;
	hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	hdr.BitRateSwitch = FDCAN_BRS_OFF;
	hdr.FDFormat = FDCAN_FD_CAN;
	hdr.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	hdr.MessageMarker = 0;

	TXdata[0] = (uint8_t)(current_scaled >> 24);
	TXdata[1] = (uint8_t)(current_scaled >> 16);
	TXdata[2] = (uint8_t)(current_scaled >> 8);
	TXdata[3] = (uint8_t)(current_scaled >> 0);

	HAL_FDCAN_AddMessageToTxFifoQ(fdcan, &hdr, TXdata);
}
