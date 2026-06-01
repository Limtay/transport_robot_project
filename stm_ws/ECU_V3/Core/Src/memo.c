/*
 * memo.c
 *
 *  Created on: 2026. 5. 31.
 *      Author: Kyeongtae
 */
  const uint32_t rs485_checking_period = 10;
  RD_RET rs485_state = RET_OK;
  uint32_t last_check_tick = osKernelGetTickCount();
  static uint8_t rs485_recovery_delay = 0;

	if (osKernelGetTickCount() - last_check_tick >= rs485_checking_period) {
		uint8_t rs485_lc = ECU_rs485.uart_obj->error.state.bits.lifecycle;
		if (rs485_lc != LS_RECOVERING || rs485_lc != LS_OFFLINE) {
			rs485_state = RD_SYSTEM_CHECKER(RD_RS485_CHECKER(&ECU_rs485, DEGRADED_K_100HZ), &hw_error.uart2, &rs485_error_cnt);
			if (rs485_state != RET_OK) {
				LED_R_state = LED_SET;
				if (rs485_state == RET_NOK) {
				#ifdef RS485_TEST_ON
					rs485_error_cnt = 0;
					if (rs485_recovery_delay == 0) {
						RD_RS485_RECOVERY(&ECU_rs485);
						rs485_recovery_delay = 10;
					} else {
						rs485_recovery_delay--;
					}
				#else
					robot_state = SYS_STATE_FAULT;
				#endif
				} else {
					rs485_recovery_delay = 0;
				}
			}
		} else {
			LED_R_state = LED_SET;
		}
		last_check_tick = osKernelGetTickCount();
	}


	if (osKernelGetTickCount() - last_check_tick >= rc_checking_period) {
		uint8_t lc = ECU_uart1.error.state.bits.lifecycle;
		if (lc == LS_RUNNING || lc == LS_DEGRADED || lc == LS_OFFLINE) {
			RD_RET rc_chk = RD_UART_CHECKER(&ECU_uart1, DEGRADED_K_100HZ);
			lc = ECU_uart1.error.state.bits.lifecycle;
			if (lc != LS_RUNNING && lc != LS_DEGRADED) {
				ECU_receive.receive_flag = 0;
			}
			if (rc_chk == RET_NOK) {
				if (rc_recovery_delay == 0) {
					RD_UART_RECOVERY(&ECU_uart1);
					rc_recovery_delay = 10;
				} else {
					rc_recovery_delay--;
				}
			} else {
				rc_recovery_delay = 0;
			}
		}
		last_check_tick = osKernelGetTickCount();
	}
