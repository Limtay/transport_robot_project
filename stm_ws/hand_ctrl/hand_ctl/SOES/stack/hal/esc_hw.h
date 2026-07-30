/*
 * esc_hw.h
 *
 *  Created on: Apr 7, 2026
 *      Author: abc01
 */

#ifndef STACK_HAL_ESC_HW_H_
#define STACK_HAL_ESC_HW_H_

#include <stdint.h>

/* soes esc config */
#include "esc.h"

/* ===== Public API ===== */

/* ESC init */
void ESC_init (const esc_cfg_t * config);

/* ESC reset */
void ESC_reset (void); //for hardware control

/* ESC read */
void ESC_read (uint16_t address, void *buf, uint16_t len);

/* ESC write */
void ESC_write (uint16_t address, void *buf, uint16_t len);

uint32_t lan9252_read_32 (uint32_t address); //static 임시로 제거함

#endif /* STACK_HAL_ESC_HW_H_ */
