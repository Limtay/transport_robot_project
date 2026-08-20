/*
 * rd_map_dpca.h
 *
 *  Created on: Jan 22, 2026
 *      Author: abc01
 */

#ifndef INC_RD_MAP_DPCA_H_
#define INC_RD_MAP_DPCA_H_

/* Private includes ----------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"
#include "rd_peripheral_dpca.h"
#include "rd_comm_dpca.h"

#include "rd_common.h"
#include "rd_define.h"
/* Exported macro ------------------------------------------------------------*/


/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

RD_RET RD_MAP_READ(PERIPHERAL_t* peripheral_obj, PACKET_comm_t *packet_obj);
RD_RET RD_MAP_WRITE(PERIPHERAL_t* peripheral_obj, PACKET_comm_t *packet_obj);

#endif /* INC_RD_MAP_DPCA_H_ */
