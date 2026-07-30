/*
 * rd_aft150_sc.c
 *
 *  Created on: 2026. 4. 28.
 *      Author: SHJ
 */


#include "rd_aft150_sc.h"

// 변수 정의

extern CAN_HandleTypeDef hcan1; // main.c에서 생성된 CAN 핸들러 참조
extern AFT150_DATA_t AFT150_data;

void Aidin_Sensor_Start(void)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[4];
    uint32_t TxMailbox;

    TxHeader.StdId = 0x220;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = 4;
    TxHeader.TransmitGlobalTime = DISABLE;

    // 1. INT CAN2.0 std mode 설정
    TxData[0] = 0x30; TxData[1] = 0x02; TxData[2] = 0x04; TxData[3] = 0x01;

    HAL_Delay(500);
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0)
    {
        HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
    }

    // 2. 온도 보상 모드 설정
    TxData[0] = 0x30; TxData[1] = 0x02; TxData[2] = 0x03; TxData[3] = 0x02;

    HAL_Delay(500);
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0)
    {
        HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
    }
}

// 센서 캘리브레이션 (Offset 제거)
void Aidin_Sensor_Cali(void)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[3];
    uint32_t TxMailbox;

    TxHeader.StdId = 0x220;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.DLC = 3;
    TxHeader.TransmitGlobalTime = DISABLE;

    TxData[0] = 0x30; TxData[1] = 0x02; TxData[2] = 0x02;

    HAL_Delay(500);
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0)
    {
        HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];

    if (hcan->Instance == hcan1.Instance)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK)
        {
            if (RxHeader.StdId == 0x230) // Force (Fx, Fy, Fz)
            {
            	AFT150_data.FT_Low[0] = (int16_t)(RxData[1] << 8 | RxData[0]);
            	AFT150_data.FT_Low[1] = (int16_t)(RxData[3] << 8 | RxData[2]);
            	AFT150_data.FT_Low[2] = (int16_t)(RxData[5] << 8 | RxData[4]);

            	AFT150_data.FT_Data[0] = (float)AFT150_data.FT_Low[0] / 100.0f;
            	AFT150_data.FT_Data[1] = (float)AFT150_data.FT_Low[1] / 100.0f;
            	AFT150_data.FT_Data[2] = (float)AFT150_data.FT_Low[2] / 100.0f;
            }
            else if (RxHeader.StdId == 0x231) // Moment (Mx, My, Mz)
            {
            	AFT150_data.FT_Low[3] = (int16_t)(RxData[1] << 8 | RxData[0]);
            	AFT150_data.FT_Low[4] = (int16_t)(RxData[3] << 8 | RxData[2]);
            	AFT150_data.FT_Low[5] = (int16_t)(RxData[5] << 8 | RxData[4]);

            	AFT150_data.FT_Data[3] = (float)AFT150_data.FT_Low[3] / 1000.0f;
            	AFT150_data.FT_Data[4] = (float)AFT150_data.FT_Low[4] / 1000.0f;
            	AFT150_data.FT_Data[5] = (float)AFT150_data.FT_Low[5] / 1000.0f;
            }
        }
    }
}
