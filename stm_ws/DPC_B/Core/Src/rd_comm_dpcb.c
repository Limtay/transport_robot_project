/*
 * rd_comm_dpcb.c
 *
 *  Created on: Jan 26, 2026
 *      Author: abc01
 */



/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "rd_comm_dpcb.h"
#include <string.h>

/* Exported includes ----------------------------------------------------------*/

/* Exported typedef -----------------------------------------------------------*/

/* Exported define ------------------------------------------------------------*/


/* Exported variables ---------------------------------------------------------*/


/* Exported function prototypes -----------------------------------------------*/
RD_RET RD_PACKET_INIT(PACKET_comm_t *packet_obj);
RD_RET RD_PACKET_READ(UART_Ring_t *uart_obj, PACKET_comm_t *packet_obj);
RD_RET RD_PACKET_WRITE(UART_Ring_t *uart_obj, PACKET_comm_t *packet_obj);

/* Private user code ---------------------------------------------------------*/
RD_RET RD_PACKET_INIT(PACKET_comm_t *packet_obj)
{
	if (packet_obj == NULL) return RET_NOK;

	packet_obj->rx.Header = 0;
	memset(packet_obj->rx.Data,0,DATA_LENGTH);
	packet_obj->rx.Checksum = 0;

	packet_obj->tx.Header = 0;
	memset(packet_obj->tx.Data,0,DATA_LENGTH);
	packet_obj->tx.Checksum = 0;

    return RET_OK;
}

RD_RET RD_PACKET_READ(UART_Ring_t *uart_obj, PACKET_comm_t *packet_obj)
{
    if (uart_obj == NULL || packet_obj == NULL) return RET_NOK;

    if (uart_obj->rx_new == 1 && uart_obj->rx_length == sizeof(PACKET_s_t))
    {
        uint8_t temp_sum = uart_obj->temp_buffer[0]; // Header
        for (int i = 0; i < DATA_LENGTH; i++)
        {
            temp_sum += uart_obj->temp_buffer[i + 1]; // Data[0] ~ Data[n]
        }

        uint8_t received_checksum = uart_obj->temp_buffer[sizeof(PACKET_s_t) - 1];

        if (temp_sum != received_checksum)
        {
            uart_obj->rx_new = 0; // 틀린 데이터는 버림
            return RET_WAIT;      // 체크섬 에러 반환
        }
        packet_obj->rx.Header = uart_obj->temp_buffer[0];
        memcpy(packet_obj->rx.Data, &uart_obj->temp_buffer[1], DATA_LENGTH);
        packet_obj->rx.Checksum = received_checksum;

        uart_obj->rx_new = 0; // 처리 완료
        return RET_OK;
    }

    if (HAL_GetTick() - uart_obj->last_rx_tick > COMM_TIMEOUT) return RET_NOK;

    return RET_WAIT;
}

RD_RET RD_PACKET_WRITE(UART_Ring_t *uart_obj, PACKET_comm_t *packet_obj)
{
	if (uart_obj == NULL || packet_obj == NULL) return RET_NOK;

	//if (packet_obj->tx.Header != PACKET_HEADER) return RET_NOK; //check empty package

	packet_obj->tx.Header = PACKET_HEADER;
	uint8_t temp_sum = packet_obj->tx.Header;
	for (int i = 0; i < DATA_LENGTH; i++)
	{
		temp_sum += packet_obj->tx.Data[i];  // Data[0] ~ Data[n]
	}
	packet_obj->tx.Checksum = temp_sum;

	memcpy(uart_obj->tx_buffer, (uint8_t *)&(packet_obj->tx), sizeof(PACKET_s_t));
	HAL_UART_Transmit_DMA(uart_obj->huart, uart_obj->tx_buffer, PACKET_LENGTH);

	return RET_OK;
}

/* ==================== READ ME ====================*/
// 0. RD_PACKET_INIT은 페리페럴적인 초기화는 없고 패킷데이터들을 초기화하는 기능을 수행함. 안해줘도 치명적인 에러는 안남.
// 1. RD_PACKET_Read은 UART_Ring_t를 통해 수신한 rx_buffer인 'IDLE사이의 데이터' 무결성을 검증한뒤, 유의미한 패킷의 형태로 PACKET_s_t안에 rx에 저장함.
// 2. RD_PACKET_Write는 PACKET_s_t안에 저장된 패킷을 UART_Ring_t안에 tx_buffer에 저장해준뒤, 실제로 Write까지 수행함.
