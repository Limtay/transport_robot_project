/*
 * rd_comm_recive.c
 *
 *  Created on: 2026. 2. 14.
 *      Author: Kyeongtae
 */



/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "rd_comm_receive.h"
#include <string.h>

/* Exported includes ----------------------------------------------------------*/

/* Exported typedef -----------------------------------------------------------*/

/* Exported define ------------------------------------------------------------*/


/* Exported variables ---------------------------------------------------------*/

/* Exported function prototypes -----------------------------------------------*/
RD_RET RD_RECEIVE_INIT(RECEIVE_comm_t *receive_obj);
RD_RET RD_RECEIVE_READ(UART_Ring_t *uart_obj, RECEIVE_comm_t *receive_obj);

/* Private user code ---------------------------------------------------------*/
RD_RET RD_RECEIVE_INIT(RECEIVE_comm_t *receive_obj)
{
    // 1. 포인터 유효성 검사 (안전벨트)
    if (receive_obj == NULL) return RET_NOK;
    memset(receive_obj, 0, sizeof(*receive_obj));
    return RET_OK;
}

RD_RET RD_RECEIVE_READ(UART_Ring_t *uart_obj, RECEIVE_comm_t *receive_obj)
{
    if (uart_obj == NULL || receive_obj == NULL) return RET_NOK;
    // 3. 신규 데이터 수신 확인
    if (uart_obj->rx_new == 0 || uart_obj->rx_length != RECEIVE_PACKET_SIZE) {
        uart_obj->rx_new = 0;
        return RET_WAIT;
    }

    uint8_t *pBuf = uart_obj->temp_buffer;
    /* --- [Step 1] Header 유효성 검사 --- */
    if (pBuf[0] != RECEIVE_HEADER1 || pBuf[1] != RECEIVE_HEADER2)
    {
    	uart_obj->comm_err_flag |= COMM_ERR_FRAMING_BIT;  /* M-2: CHECKER 에 framing 에러 통보 */
    	uart_obj->rx_new = 0;
    	return RET_WAIT;
    }
    /* --- [Step 2] Checksum 계산 --- */
    uint16_t calc_sum = 0;
    int check_idx = RECEIVE_PACKET_SIZE - 2;
    for (int i = 0; i < check_idx; i++) {
    	calc_sum += pBuf[i];
    }
    calc_sum = ~calc_sum;

    /* --- [Step 3] Checksum 비교 --- */
    uint16_t received_checksum = (uint16_t)(pBuf[check_idx] | (pBuf[check_idx+1] << 8));
    if (calc_sum != received_checksum) {
    	uart_obj->comm_err_flag |= COMM_ERR_CRC_BIT;  /* M-2: CHECKER 에 CRC 에러 통보 */
    	uart_obj->rx_new = 0;
    	return RET_WAIT;
    }
    /* --- [Step 4] 데이터 복사 --- */
    memcpy(&(receive_obj->packet), pBuf, RECEIVE_PACKET_SIZE);

    /* --- [Step 5] 12비트 클리핑 및 조종기 데이터 가공 --- */
    for (int i = 0; i < RECEIVE_DATA_CHANNELS; i++) {
    	// 구조체에 예쁘게 들어간 값에 마스크만 씌워줌
    	receive_obj->packet.Data[i] &= 0x0FFF;
    }
    receive_obj->thrr1 = ((int)receive_obj->packet.Data[1] - 1500) * 14;
    receive_obj->diff1 = ((int)receive_obj->packet.Data[0] - 1500) * 6;

    receive_obj->thrr2 = ((int)receive_obj->packet.Data[2] - 1500) * 14;
    receive_obj->diff2 = ((int)receive_obj->packet.Data[3] - 1500) * 6;

    receive_obj->receive_flag = (receive_obj->packet.Data[7] == 1000) ? 1 : 0;

    /*TODO: mode 변경 임시플래그*/
    receive_obj->mode_flag    = (receive_obj->packet.Data[4] == 1000) ? 0 : 1;

    for (int i = 0; i < 2; i++) {
    	switch (receive_obj->packet.Data[5+i]) {
    	case 2000:
    		receive_obj->selector[i] = 1;
    		break;
    	case 1500:
    		receive_obj->selector[i] = 2;
    		break;
    	case 1000:
    		receive_obj->selector[i] = 3;
    		break;
    	default :
    		receive_obj->selector[i]  = 0;
    		break;
    	}
    }

    /* lifecycle 전이는 Checker 가 소유 — rx_new 만 클리어 */
	uart_obj->rx_new = 0;
    return RET_OK;
}

RD_CONTROL_RC_TO_REGISTER(&ECU_receive, &reg.cmd_motor);
