/*
 * rd_map_dpca.c
 *
 *  Created on: Jan 22, 2026
 *      Author: abc01
 */

/* Includes ------------------------------------------------------------------*/
#include "rd_map_dpca.h"
#include <string.h>

/* Exported includes
 * ----------------------------------------------------------*/

/* Exported typedef
 * -----------------------------------------------------------*/

/* Exported define
 * ------------------------------------------------------------*/

/* Exported variables
 * ---------------------------------------------------------*/

/* Exported function prototypes
 * -----------------------------------------------*/

RD_RET RD_MAP_READ(PERIPHERAL_t *peripheral_obj, PACKET_comm_t *packet_obj);
RD_RET RD_MAP_WRITE(PERIPHERAL_t *peripheral_obj, PACKET_comm_t *packet_obj);

/* Private user code ---------------------------------------------------------*/

RD_RET RD_MAP_READ(PERIPHERAL_t *peripheral_obj, PACKET_comm_t *packet_obj) {
  if (peripheral_obj == NULL || packet_obj == NULL)
    return RET_NOK;

  uint8_t *data = packet_obj->rx.Data;

  peripheral_obj->EN_ALL = (data[0] != 0) ? 1 : 0;
  peripheral_obj->EN_BOOT = (data[1] != 0) ? 1 : 0;

  return RET_OK;
}

RD_RET RD_MAP_WRITE(PERIPHERAL_t *peripheral_obj, PACKET_comm_t *packet_obj) {
  if (peripheral_obj == NULL || packet_obj == NULL)
    return RET_NOK;

  packet_obj->tx.Data[0] = ((peripheral_obj->CON_A != 0) ? 1 << 0 : 0) |
                           ((peripheral_obj->CON_B != 0) ? 1 << 1 : 0) |
                           ((peripheral_obj->CON_C != 0) ? 1 << 2 : 0) |
                           ((peripheral_obj->CON_D != 0) ? 1 << 3 : 0);

  packet_obj->tx.Data[1] = ((peripheral_obj->PXS_A != 0) ? 1 << 0 : 0) |
                           ((peripheral_obj->PXS_B != 0) ? 1 << 1 : 0) |
                           ((peripheral_obj->PXS_C != 0) ? 1 << 2 : 0);

  return RET_OK;
}

/* ==================== READ ME ====================*/
// 0. 초기화 여기는 없는데 원래는 맵 초기화 기능들어가야함. 이건 컴팩트한 맵이라
// 걍 안한거
// 1. RD_MAP_Read는 packet_obj를 peripheral_obj에 매핑해주는 기능.
// 2. RD_MAP_Write는 현재 peripheral_obj의 값을 packet_obj에 매핑해주는 기능.
