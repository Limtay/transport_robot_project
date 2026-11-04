/*
 * comm_dummy_rls.c
 *
 *  Created on: 2026. 11. 4.
 *      Author: SHJ
 */

#include "comm_dummy_rls.h"
#include "incremental_enc.h"

/* Macro ---------------------------------------------------------------------*/
//typedef uint32_t RD_RET;

//#define RET_OK ((RD_RET)0x00)
//#define RET_NOK ((RD_RET)0x01)
//#define RET_WAIT ((RD_RET)0x02)

/* Exported variables --------------------------------------------------------*/
extern LinearEncoder_t ENC[2];
extern UART_HandleTypeDef huart1;

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

/* Private variables ---------------------------------------------------------*/
uint8_t rx_data;             // 1바이트 수신 전용 버퍼
uint8_t tx_packet[8];        // 8바이트 DMA 송신 패킷 버퍼

/* Private function ----------------------------------------------------------*/
void UART_RLS_INIT(){
	HAL_UART_Receive_IT(&huart1, &rx_data, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
    	if (rx_data == '3'){
    		memcpy(&tx_packet[0], &ENC[0].accumulated_count, 4);
    	    memcpy(&tx_packet[4], &ENC[1].accumulated_count, 4);

    	    HAL_UART_Transmit_DMA(&huart1, tx_packet, 8);
    	}

    	else if (rx_data == 'i'){
    		ENC_SET_ZERO(&ENC[0], &htim3);
    		ENC_SET_ZERO(&ENC[1], &htim4);

    		tx_packet[0] = 0; tx_packet[1] = 0; tx_packet[2] = 0; tx_packet[3] = 0;
    		tx_packet[4] = 0; tx_packet[5] = 0; tx_packet[6] = 0; tx_packet[7] = 0;

    		HAL_UART_Transmit_DMA(&huart1, tx_packet, 8);
    	}

    	else{
    		tx_packet[0] = 255; tx_packet[1] = 255; tx_packet[2] = 255; tx_packet[3] = 255;
    		tx_packet[4] = 255; tx_packet[5] = 255; tx_packet[6] = 255; tx_packet[7] = 255;

    		HAL_UART_Transmit_DMA(&huart1, tx_packet, 8);
    	}

    	HAL_UART_Receive_IT(&huart1, &rx_data, 1);

    }
}
