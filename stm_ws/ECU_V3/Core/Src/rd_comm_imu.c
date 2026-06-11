/*
 * rd_comm_imu.c
 *
 *  Created on: Jun 10, 2026
 *      Author: swarm
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
RD_RET RD_IMU_INIT(IMU_comm_t *imu_obj);
RD_RET RD_IMU_READ(UART_Ring_t *uart_obj, IMU_comm_t *imu_obj);

/* Private user code ---------------------------------------------------------*/
RD_RET RD_IMU_INIT(IMU_comm_t *imu_obj)
{
    // 1. 포인터 유효성 검사 (안전벨트)
    if (imu_obj == NULL) return RET_NOK;
    memset(imu_obj, 0, sizeof(*imu_obj));
    return RET_OK;
}

RD_RET RD_IMU_READ(UART_Ring_t *uart_obj, IMU_comm_t *imu_obj)
{
    if (uart_obj == NULL || imu_obj == NULL) return RET_NOK;
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
    memcpy(&(imu_obj->packet), pBuf, RECEIVE_PACKET_SIZE);

    /* --- [Step 5] 12비트 클리핑 및 조종기 데이터 가공 --- */
    for (int i = 0; i < RECEIVE_DATA_CHANNELS; i++) {
    	// 구조체에 예쁘게 들어간 값에 마스크만 씌워줌
    	imu_obj->packet.Data[i] &= 0x0FFF;
    }

    /* lifecycle 전이는 Checker 가 소유 — rx_new 만 클리어 */
	uart_obj->rx_new = 0;
    return RET_OK;
}
