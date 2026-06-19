/*
 * rd_map_dyn.c
 *  Dynamixel 매핑 계층 – DYN_Ctrl_t ↔ 패킷 변환 (고수준)
 *
 *  Created on: Mar 10, 2026
 *      Author: swarm
 */

/* Includes ------------------------------------------------------------------*/
#include "rd_map_dyn.h"
#include <string.h>

/* Exported user code --------------------------------------------------------*/

RD_RET RD_DYN_INIT(DYN_Ctrl_t *ctrl, uint8_t id)
{
    if (ctrl == NULL) return RET_NOK;

    memset(ctrl, 0, sizeof(DYN_Ctrl_t));
    ctrl->id = id;

    return RET_OK;
}

RD_RET RD_DYN_INIT_SET(RS485_t *rs485_obj, DYN_Ctrl_t *ctrl)
{
    if (ctrl == NULL) return RET_NOK;
    /*----- Torque off ----- */
    ctrl->inst       = INST_WRITE;
    ctrl->enable	 = 0;
    ctrl->addr.start = DYN_ADDR_TORQUE_ENABLE;
    ctrl->addr.size  = DYN_SIZE_TORQUE_ENABLE;
    if (RD_DYN_LOOP(rs485_obj, ctrl) != RET_OK) return RET_WAIT;

    /*-----Operating Mode Read ----- */
    ctrl->inst       = INST_READ;
    ctrl->addr.start = DYN_ADDR_OPERATING_MODE;
    ctrl->addr.size  = DYN_SIZE_OPERATING_MODE;
    if (RD_DYN_LOOP(rs485_obj, ctrl) != RET_OK) return RET_WAIT;
    ctrl->pre_mode = ctrl->mode;
    return RET_OK;
}

RD_RET RD_DYN_READ(RS485_t *rs485_obj, DYN_Ctrl_t *ctrl)
{
    if (ctrl == NULL || rs485_obj == NULL) return RET_NOK;

    if (RD_DYNPACK_READ(rs485_obj, &ctrl->comm) != RET_OK) return RET_WAIT;
    /* 수신된 Instruction이 Status가 아니면 무시 */
    if (ctrl->comm.rx.Instruction != INST_STATUS) return RET_WAIT;
    /* 수신된 ID가 현재 id와 일치하지 않으면 무시 */
    if (ctrl->comm.rx.TargetID    != ctrl->id)    return RET_WAIT;
    /* Status Error Parsing + Error 발생 시 무시 */
    ctrl->error.status_error = ctrl->comm.rx.Data[0];
    if (ctrl->error.status_error != 0) return RET_WAIT;

    switch (ctrl->inst) {
        case INST_PING:
            break;
        case INST_READ: {
        	/*------------ Enable/Mode READ-----------------*/
        	if (ctrl->addr.start == DYN_ADDR_TORQUE_ENABLE)
        		ctrl->enable = ctrl->comm.rx.Data[1];
        	else if (ctrl->addr.start == DYN_ADDR_OPERATING_MODE)
        		ctrl->mode	 = ctrl->comm.rx.Data[1];
        	/*-------------- RAM DATA READ-------------------*/
        	else {
				uint8_t  *pBuf      = (uint8_t *)&ctrl->ram;
				uint16_t  addr      = ctrl->addr.start - DYN_CMD_START_ADDR;
				uint16_t  data_size = ctrl->comm.rx.data_len - 1; /* Error(1) 제외 */

				if (data_size != ctrl->addr.size) {
					ctrl->error.debug_cnt1++;
					return RET_WAIT;
				}
				memcpy(&pBuf[addr], &ctrl->comm.rx.Data[1], data_size);
        	}
            break;
        }
        case INST_WRITE:
		case INST_REBOOT:
			break;
        default:
            return RET_WAIT;
    }
    ctrl->is_running = 1;
    ctrl->error.comm_flag = 0;
    return RET_OK;
}

RD_RET RD_DYN_WRITE(RS485_t *rs485_obj, DYN_Ctrl_t *ctrl)
{
    if (ctrl == NULL || rs485_obj == NULL) return RET_NOK;

    ctrl->error.comm_flag     = 1;
    ctrl->comm.tx.TargetID    = ctrl->id;
    ctrl->comm.tx.Instruction = ctrl->inst;

    switch (ctrl->inst) {
        case INST_PING:
            ctrl->comm.tx.data_len = 0;
            break;
        case INST_READ:
            /* Parameter: Start Address(2) + Data Length(2) = DYN_RAM_ACCESS_t 그대로 */
            memcpy(ctrl->comm.tx.Data, &ctrl->addr, sizeof(DYN_RAM_ACCESS_t));
            ctrl->comm.tx.data_len = 4;
            break;
        case INST_WRITE:
            /* Parameter: Start Address(2) + Write Data(N) */
            ctrl->comm.tx.Data[0] = ctrl->addr.start & 0xFF;
            ctrl->comm.tx.Data[1] = (ctrl->addr.start >> 8) & 0xFF;
            ctrl->comm.tx.data_len = ctrl->addr.size + 2;

            switch (ctrl->addr.start) {
                case DYN_ADDR_TORQUE_ENABLE:
                    ctrl->comm.tx.Data[2] = ctrl->enable;
                    break;
                case DYN_ADDR_OPERATING_MODE:
                    ctrl->comm.tx.Data[2] = (uint8_t)ctrl->mode;
                    break;
                default: {
                    uint8_t  *pBuf = (uint8_t *)&ctrl->ram;
                    uint16_t  addr = ctrl->addr.start - DYN_CMD_START_ADDR;
                    memcpy(&ctrl->comm.tx.Data[2], &pBuf[addr], ctrl->addr.size);
                    break;
                }
            }
            break;
		case INST_REBOOT:
			ctrl->comm.tx.data_len = 0;
			break;
        default:
            return RET_WAIT;
    }
    return RD_DYNPACK_WRITE(rs485_obj, &ctrl->comm);
}

RD_RET RD_DYN_CHECK(DYN_Ctrl_t *ctrl)
{
    if (ctrl == NULL) return RET_NOK;

    if (ctrl->error.comm_flag) {
        ctrl->error.comm_cnt++;
        switch (ctrl->addr.start) {
            case DYN_ADDR_TORQUE_ENABLE:
                ctrl->enable = !ctrl->enable;
                break;
            case DYN_ADDR_OPERATING_MODE:
                ctrl->mode = ctrl->pre_mode;
                break;
        }
        return RET_NOK;
    }
    return RET_OK;
}

RD_RET RD_DYN_LOOP(RS485_t *rs485_obj, DYN_Ctrl_t *ctrl) {
	if (RD_DYN_WRITE(rs485_obj, ctrl) != RET_OK) return RET_NOK;
#ifdef RTOS_IS_AVAILABLE
	osThreadFlagsWait(0x0001, osFlagsWaitAny, 10);
#else
	HAL_Delay(10);
#endif
	if (RD_DYN_READ(rs485_obj, ctrl) != RET_OK) return RET_NOK;
	return RD_DYN_CHECK(ctrl);
}

RD_RET RD_DYN_OPERATE_ON(DYN_Ctrl_t *ctrl, DYN_MODE_e mode)
{
    if (ctrl == NULL) return RET_NOK;

    ctrl->inst = INST_WRITE;
    if (ctrl->mode != mode) {
        ctrl->mode           = mode;
        ctrl->addr.start     = DYN_ADDR_OPERATING_MODE;
        ctrl->addr.size      = DYN_SIZE_OPERATING_MODE;
        return RET_WAIT;
    }
    if (!ctrl->enable) {
        ctrl->enable         = 1;
        ctrl->addr.start     = DYN_ADDR_TORQUE_ENABLE;
        ctrl->addr.size      = DYN_SIZE_TORQUE_ENABLE;
        return RET_WAIT;
    }
    return RET_OK;
}

RD_RET RD_DYN_UPDATE_CMD(DYN_Ctrl_t *ctrl, DYN_MODE_e mode)
{
    if (ctrl == NULL) return RET_NOK;

    ctrl->inst = INST_WRITE;
    switch (mode) {
        case DYN_MODE_CURRENT:
            ctrl->addr.start = DYN_ADDR_GOAL_CURRENT;
            ctrl->addr.size  = DYN_SIZE_GOAL_CURRENT;
            break;
        case DYN_MODE_VELOCITY:
            ctrl->addr.start = DYN_ADDR_GOAL_VELOCITY;
            ctrl->addr.size  = DYN_SIZE_GOAL_VELOCITY;
            break;
        case DYN_MODE_POSITION:
        case DYN_MODE_EXT_POSITION:
        case DYN_MODE_CUR_POSITION:
            ctrl->addr.start = DYN_ADDR_GOAL_POSITION;
            ctrl->addr.size  = DYN_SIZE_GOAL_POSITION;
            break;
        default:
            return RET_NOK;
    }
    return RET_OK;
}

RD_RET RD_DYN_UPDATE_STATE(DYN_Ctrl_t *ctrl)
{
    if (ctrl == NULL) return RET_NOK;

    ctrl->inst       = INST_READ;
    ctrl->addr.start = DYN_STATE_START_ADDR;
    ctrl->addr.size  = DYN_STATE_SIZE;
    return RET_OK;
}
