/*
 * rd_map_dyn.h
 *  Dynamixel 매핑 계층 – DYN_Ctrl_t ↔ 패킷 변환 (고수준)
 *
 *  Created on: Mar 10, 2026
 *      Author: swarm
 */

#ifndef INC_RD_MAP_DYN_H_
#define INC_RD_MAP_DYN_H_

/* Private includes ----------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "rd_common.h"
#include "rd_define.h"

#include "rd_uart.h"
#include "rd_comm_dyn.h"

#include "DYN_xm430_w350.h"

/* ── 모터 단위 데이터 구조체 ─────────────────────────────────────────────────*/
typedef enum {
    /* 기본 명령어 */
    INST_PING              = 0x01, // 장치가 네트워크에 연결되어 있는지 확인 (Model No, FW Version 반환)
    INST_READ              = 0x02, // 컨트롤 테이블의 특정 주소에서 데이터를 읽음
    INST_WRITE             = 0x03, // 컨트롤 테이블의 특정 주소에 데이터를 씀
    INST_REG_WRITE         = 0x04, // Write와 유사하지만, ACTION 명령이 올 때까지 대기함
    INST_ACTION            = 0x05, // REG_WRITE로 대기 중인 명령을 실행시킴
    INST_FACTORY_RESET     = 0x06, // 장치의 설정을 공장 출하 상태로 초기화
    INST_REBOOT            = 0x08, // 장치를 재시작 (Error 상태 해제 등에 사용)
    INST_CLEAR             = 0x10, // 특정 상태(예: Multi-turn 정보)를 초기화
    INST_CONTROL_TABLE_BACKUP = 0x20, // 현재 컨트롤 테이블 값을 백업 메모리에 저장

    /* 응답 관련 */
    INST_STATUS            = 0x55, // 장치가 마스터에게 보내는 응답 패킷 (Status Packet)

    /* 다중 제어용 (Sync) - 동일한 주소에 접근할 때 사용 */
    INST_SYNC_READ         = 0x82, // 여러 장치로부터 동시에 데이터를 읽음
    INST_SYNC_WRITE        = 0x83, // 여러 장치에 동시에 데이터를 씀 (예: 여러 모터 Goal Position 동시 제어)
    INST_FAST_SYNC_READ    = 0x8A, // SYNC_READ보다 응답 속도가 빠른 읽기 명령 (주로 사용 예정)

    /* 다중 제어용 (Bulk) - 서로 다른 주소에 접근할 때 사용 */
    INST_BULK_READ         = 0x92, // 여러 장치(다른 주소/길이)에서 데이터를 읽음
    INST_BULK_WRITE        = 0x93, // 여러 장치(다른 주소/길이)에 데이터를 씀
    INST_FAST_BULK_READ    = 0x9A  // BULK_READ보다 응답 속도가 빠른 읽기 명령
} DYN_INST_e;

typedef struct {
    uint8_t  comm_flag;
    uint16_t comm_cnt;
    DYN_STATUS_ERROR_e  status_error;

    uint16_t debug_cnt1;
    uint16_t debug_cnt2;
}DYN_ERROR_t;

typedef struct __attribute__((packed)){
    uint16_t     start;
    uint16_t     size;
}DYN_RAM_ACCESS_t;

typedef struct __attribute__((packed)) {
    /* addr 100 */ DYN_CMD_t   cmd;      // RW
    /* addr 120 */ DYN_STATE_t state;    // RO
}DYN_RAM_s_t;

typedef struct {
    uint8_t     id;
    uint8_t     enable;
    uint8_t     is_running;

    DYN_INST_e  inst;

    DYN_MODE_e  pre_mode;
    DYN_MODE_e  mode;

    DYN_RAM_ACCESS_t addr;
    DYN_RAM_s_t ram;
    
    DYN_comm_t  comm;
    DYN_ERROR_t error;
} DYN_Ctrl_t;

/* Exorted functions prototypes ---------------------------------------------*/

/** @brief 구조체 초기화 (모터 ID, 데이터 0 클리어) */
RD_RET RD_DYN_INIT(DYN_Ctrl_t *ctrl, uint8_t id);
RD_RET RD_DYN_INIT_SET(RS485_t *rs485_obj, DYN_Ctrl_t *ctrl);

/** @brief Status 패킷 수신 및 ram 구조체 갱신 */
RD_RET RD_DYN_READ(RS485_t *rs485_obj, DYN_Ctrl_t *ctrl);
/** @brief inst/addr 설정에 따라 Write/Read 패킷 송신 */
RD_RET RD_DYN_WRITE(RS485_t *rs485_obj, DYN_Ctrl_t *ctrl);
/** @brief comm_flag 확인 및 Write 실패 시 롤백 */
RD_RET RD_DYN_CHECK(DYN_Ctrl_t *ctrl);
/** @brief READ + Write + Check Loop */
RD_RET RD_DYN_LOOP(RS485_t *rs485_obj, DYN_Ctrl_t *ctrl);

/** @brief Torque ON → Operating Mode 설정 (순서 보장) */
RD_RET RD_DYN_OPERATE_ON(DYN_Ctrl_t *ctrl, DYN_MODE_e mode);
/** @brief 제어 모드에 맞는 Goal 레지스터 주소/크기 설정 */
RD_RET RD_DYN_UPDATE_CMD(DYN_Ctrl_t *ctrl, DYN_MODE_e mode);
/** @brief Present 상태 레지스터 읽기 주소/크기 설정 */
RD_RET RD_DYN_UPDATE_STATE(DYN_Ctrl_t *ctrl);

#endif /* INC_RD_MAP_DYN_H_ */
