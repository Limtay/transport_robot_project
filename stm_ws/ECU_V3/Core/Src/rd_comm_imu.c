/*
 * rd_comm_imu.c
 *
 *  EBIMU-9DOFV6 HEX(binary) 패킷 수신 레이어 — 패킷 포맷/설정은 rd_comm_imu.h 참고.
 *  규칙은 rd_comm_receive.c (UART1 RC) 와 동일: 검증 실패는 comm_err_flag 통보 후
 *  RET_WAIT, lifecycle 전이는 Checker 소유.
 *
 *  Created on: Jun 10, 2026
 *      Author: swarm
 */

/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "rd_comm_imu.h"
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
    // 신규 데이터 수신 확인 (IDLE 1회 = 패킷 1개, 250Hz/4ms 간격)
    if (uart_obj->rx_new == 0 || uart_obj->rx_length != IMU_PACKET_SIZE) {
        uart_obj->rx_new = 0;
        return RET_WAIT;
    }

    uint8_t *pBuf = uart_obj->temp_buffer;
    /* --- [Step 1] Header(SOP) 유효성 검사 --- */
    if (pBuf[0] != IMU_HEADER1 || pBuf[1] != IMU_HEADER2)
    {
    	uart_obj->comm_err_flag |= COMM_ERR_FRAMING_BIT;  /* CHECKER 에 framing 에러 통보 */
    	uart_obj->rx_new = 0;
    	return RET_WAIT;
    }
    /* --- [Step 2] Checksum 계산 (SOP 포함 모든 byte 합, overflow 무시) --- */
    uint16_t calc_sum = 0;
    int check_idx = IMU_PACKET_SIZE - 2;
    for (int i = 0; i < check_idx; i++) {
    	calc_sum += pBuf[i];
    }

    /* --- [Step 3] Checksum 비교 (CHK 는 Big-Endian: MSB 먼저) --- */
    uint16_t received_checksum = (uint16_t)((pBuf[check_idx] << 8) | pBuf[check_idx+1]);
    if (calc_sum != received_checksum) {
    	uart_obj->comm_err_flag |= COMM_ERR_CRC_BIT;  /* CHECKER 에 CRC 에러 통보 */
    	uart_obj->rx_new = 0;
    	return RET_WAIT;
    }
    /* --- [Step 4] 데이터 복사 — Big-Endian(int16) → Little-Endian 변환만 수행.
     *     물리값 변환(×0.0001 등)은 상위단(Orin) 담당, raw 그대로 저장 --- */
    int16_t *pDst = (int16_t *)&(imu_obj->packet);
    for (int i = 0; i < IMU_DATA_CHANNELS - 1; i++) {
    	pDst[i] = (int16_t)((pBuf[2 + 2*i] << 8) | pBuf[3 + 2*i]);
    }
    /* timestamp 는 uint16 (0~60000ms 순환 — int16 범위 초과 주의) */
    imu_obj->timestamp = (uint16_t)((pBuf[22] << 8) | pBuf[23]);

    /* lifecycle 전이는 Checker 가 소유 — rx_new 만 클리어 */
	uart_obj->rx_new = 0;
    return RET_OK;
}
