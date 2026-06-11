/*
 * rd_comm_imu.h
 *
 *  EBIMU-9DOFV6 (E2BOX) HEX(binary) 출력 패킷 수신 레이어.
 *
 *  센서 설정 (etc/IMU/default_set.md — 비휘발성 저장):
 *      <sb8>  : 921600bps
 *      <sor4> : 출력 간격 4ms (250Hz)
 *      <soc2> : HEX(binary) 출력 모드
 *      <sof2> : Quaternion 출력 (z,y,x,w 순서)
 *      <sog1> : 자이로(각속도) 출력 ON
 *      <soa2> : 중력성분 제거된 Local 가속도 출력 ON
 *      <sod0> / <sot0> : 거리/온도 출력 OFF
 *      <sots1>: 타임스탬프 출력 ON
 *
 *  패킷 구조 (총 26 bytes, 모든 항목 16bit 2의 보수 Big-Endian):
 *      [0:1]   SOP        0x55 0x55
 *      [2:9]   Quaternion z,y,x,w   raw ×0.0001 [무단위]
 *      [10:15] Gyro       x,y,z     raw ×0.1    [deg/s]
 *      [16:21] Accel      x,y,z     raw ×0.001  [g] (1g = 9.81 m/s^2)
 *      [22:23] Timestamp  (uint16)  raw ×1      [ms] 0~60000 순환
 *      [24:25] CHK        SOP 포함 전체 byte 합 (overflow 무시)
 *
 *  Created on: Jun 10, 2026
 *      Author: swarm
 */

#ifndef INC_RD_COMM_IMU_H_
#define INC_RD_COMM_IMU_H_

/* Private includes ----------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "rd_common.h"
#include "rd_define.h"
#include "rd_uart.h"

/* Exported macro ------------------------------------------------------------*/
// 헤더 정의
#define IMU_HEADER1  0x55
#define IMU_HEADER2  0x55

#define IMU_PACKET_SIZE   26
#define IMU_DATA_CHANNELS 11   /* quat 4 + gyro 3 + acc 3 + timestamp 1 (int16) */

/* Exported types ------------------------------------------------------------*/

/*
 *  수신 데이터 (Big-Endian → Little-Endian 변환 후 raw 그대로 저장).
 *  멤버 순서/크기 = rd_register_ecu.h DATA_IMU_t 의 데이터부(addr 62~81)와 1:1
 *  → MARSHAL 시 memcpy 한 번으로 reg.imu 에 발행. 물리값 변환은 상위단(Orin) 담당.
 */
typedef struct __attribute__((packed)){
	int16_t quat[4];   /* z,y,x,w — raw ×0.0001 [무단위]                  */
	int16_t gyro[3];   /* x,y,z   — raw ×0.1    [deg/s]                  */
	int16_t acc[3];    /* x,y,z   — raw ×0.001  [g] (중력 제거 Local, soa2) */
} IMU_comm_s_t;

typedef struct {
	IMU_comm_s_t      packet;
	volatile uint16_t timestamp;  /* raw ×1 [ms] 0~60000 순환 (sots1) — reg 미발행, 신선도 확인용 */
} IMU_comm_t;
/* Exported constants --------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
RD_RET RD_IMU_INIT(IMU_comm_t *imu_obj);
RD_RET RD_IMU_READ(UART_Ring_t *uart_obj, IMU_comm_t *imu_obj);

#endif /* INC_RD_COMM_IMU_H_ */
