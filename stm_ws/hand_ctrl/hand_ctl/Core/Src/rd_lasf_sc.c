/*
 * rd_lasf_sc.c
 *
 *  Created on: Feb 24, 2026
 *      Author: abc01
 */


/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "rd_lasf_sc.h"
#include <string.h>

/* Exported includes ----------------------------------------------------------*/

/* Exported typedef -----------------------------------------------------------*/

/* Exported define ------------------------------------------------------------*/


/* Exported variables ---------------------------------------------------------*/


/* Exported function prototypes -----------------------------------------------*/
RD_RET RD_LASF_INIT(LASF_COMM_t *lasf_obj);
RD_RET RD_LASF_READ(UART_Ring_t *uart_obj, LASF_COMM_t *packet_obj);
RD_RET RD_LASF_WRITE(UART_Ring_t *uart_obj, LASF_COMM_t *packet_obj);
RD_RET RD_LASF_UPDATE(LASF_COMM_t *packet_obj);

/* Private user code ---------------------------------------------------------*/
RD_RET RD_LASF_INIT(LASF_COMM_t *lasf_obj)
{
	if (lasf_obj == NULL) return RET_NOK;

	//TX PACKET INIT
	lasf_obj->TX.HeaderH = PACKET_HEADER1;
	lasf_obj->TX.HeaderL = PACKET_HEADER2;
	lasf_obj->TX.DAL = DATA_LENGTH_TX;
	lasf_obj->TX.ID = 0;
	lasf_obj->TX.CMD = CMD_WR;
	lasf_obj->TX.REG_IDX = IDX_MODE;
	lasf_obj->TX.CTL_POS = 0;
	lasf_obj->TX.Checksum = 0;

	//RX PACKET INIT

	//DATA INIT
	lasf_obj->LASF.ID = 0;
	lasf_obj->LASF.CTL_POS = 0;
	lasf_obj->LASF.ACT_POS = 0;
	lasf_obj->LASF.CURRENT = 0;
	lasf_obj->LASF.FT_VAL = 0;
	lasf_obj->LASF.TEMP = 0;
	lasf_obj->LASF.Error_C = 0;

    return RET_OK;
}

RD_RET RD_LASF_READ(UART_Ring_t *uart_obj, LASF_COMM_t *packet_obj)
{
    if (uart_obj == NULL || packet_obj == NULL) return RET_NOK;

    if (uart_obj->rx_new == 1 && uart_obj->rx_length == sizeof(LASF_RX_PACKET_t))
    {
        uint8_t temp_sum = 0; // Header
        for (int i = RX_SUM_IDX1; i < RX_SUM_IDX2; i++)
        {
            temp_sum += uart_obj->temp_buffer[i]; // Data[0] ~ Data[n]
        }

        uint8_t received_checksum = uart_obj->temp_buffer[sizeof(LASF_RX_PACKET_t) - 1];

        if (temp_sum != received_checksum)
        {
            uart_obj->rx_new = 0; // 틀린 데이터는 버림
            return RET_WAIT;      // 체크섬 에러 반환
        }

        memcpy(&(packet_obj->RX), uart_obj->temp_buffer, sizeof(LASF_RX_PACKET_t));


        uart_obj->rx_new = 0; // 처리 완료
        return RET_OK;
    }

    if (HAL_GetTick() - uart_obj->last_rx_tick > RX_TIMEOUT) return RET_NOK;

    return RET_WAIT;
}

RD_RET RD_LASF_WRITE(UART_Ring_t *uart_obj, LASF_COMM_t *packet_obj)
{
	if (uart_obj == NULL || packet_obj == NULL) return RET_NOK;

	memcpy(uart_obj->tx_buffer, &(packet_obj->TX), sizeof(LASF_TX_PACKET_t));

	uint8_t temp_sum = 0;
	for (int i = TX_SUM_IDX1; i < TX_SUM_IDX2; i++)
	{
		temp_sum += uart_obj->tx_buffer[i];  // buff[star] ~ buff[n]
	}
	packet_obj->TX.Checksum = temp_sum;
	uart_obj->tx_buffer[TX_SUM_IDX2] = packet_obj->TX.Checksum;
	uart_obj->tx_length = PACKET_TX_LEN;
	RD_UART_Transmit(uart_obj);

	return RET_OK;
}
