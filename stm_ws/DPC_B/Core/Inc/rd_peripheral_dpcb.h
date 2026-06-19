/*
 * rd_peripheral_dpcb.h
 *
 *  Created on: Jan 27, 2026
 *      Author: abc01
 */

#ifndef INC_RD_PERIPHERAL_DPCB_H_
#define INC_RD_PERIPHERAL_DPCB_H_



/* Private includes ----------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"
#include "rd_common.h"
#include "rd_define.h"
#include "MCP23017.h" //test
#include "rd_comm_dpcb.h"
#include "rd_peripheral.h"
#include "rd_map_dyn.h"      /* rd_comm_dyn.h 포함 */


/* Exported macro ------------------------------------------------------------*/
// GPA 포트 (0~7)
#define EXIO_LED1    0    // GPA0
#define EXIO_LED2    1    // GPA1
#define EXIO_SW3B    2    // GPA2
#define EXIO_SW4B    3    // GPA3
#define EXIO_SW5B    4    // GPA4
#define EXIO_SW6B    5    // GPA5

// GPB 포트 (8~15)
#define EXIO_SW6A    10   // GPB2
#define EXIO_SW5A    11   // GPB3
#define EXIO_SW4A    12   // GPB4
#define EXIO_SW3A    13   // GPB5
#define EXIO_SW2     14   // GPB6
#define EXIO_SW1     15   // GPB7

#define DYN_NUM_MOTORS          3

/* Exported types ------------------------------------------------------------*/

typedef struct {
	I2C_HandleTypeDef *hi2c;			//23017 gpio expander i2c handler
	//input pin state define
	uint8_t SW1_state;					//SPST x2 		: 0 is off, 1 is on
	uint8_t SW2_state;

	uint8_t SW3_state;					//SPDT x4 		: 0 is mid, 1 is up, 2 is down
	uint8_t SW4_state;
	uint8_t SW5_state;
	uint8_t SW6_state;

	//output pin state define
	uint8_t LED1_state;					//SPST LED x2	: 0 is off, 1 is on
	uint8_t LED2_state;
} peripheral_exio_t;


typedef struct {
	GPIO_IO_t BOOT_IO_1;
	GPIO_IO_t BOOT_IO_2;

	GPIO_IO_t EN_IO;
	GPIO_IO_t CON_A_IO;
	GPIO_IO_t CON_B_IO;
	GPIO_IO_t CON_C_IO;
	GPIO_IO_t CON_D_IO;

	GPIO_IO_t SERVO_IO; 			//추가한거.
	GPIO_IO_t LIGHT_IO;

} PERIPHERAL_IO_ALL_t;


typedef struct {

	volatile int32_t CTL_SPEED;		// Calc speed same as dyn
	volatile int32_t INIT_POS;		// init base position when init seq
	volatile int32_t TARGET_POS;	// force drive pos when deploy
	volatile int16_t LPF_CURRENT;	// lpf current

	uint8_t DYN_IDS;
	DYN_Ctrl_t dyn_ctrl;

	uint16_t dyn_present_tick;
	uint16_t delta_tick;

} PERIPHERAL_MOT_t;


typedef struct {
	/*==================== DPC_A data ====================*/
	volatile uint8_t A_EN_ALL;
	volatile uint8_t A_EN_BOOT;

	volatile uint8_t A_CON_DATA;		// 1 byte but use 4 bit. 1 is lock, 0 unlock
	volatile uint8_t A_PROX_DATA;		// 1 byte but use 3 bit

	/*==================== DPC_B data ====================*/
	//Solenoid CTRL
	volatile uint8_t EN_ALL;			//All solenoid enable
	volatile uint32_t LAST_EN_TICK;		//Solenoid timeout
	volatile uint8_t EN_BOOT;

	volatile uint8_t CON_DATA;

	volatile uint8_t SERVO_EN;			//Servo active. 0: idle, 1: unlock, 2:
	volatile uint8_t LIGHT_EN;			//LIGHT LED active. 0: off, 1:on

	volatile uint8_t B_EN_BOOT;

	//Winch CTRL
	PERIPHERAL_MOT_t MOT[DYN_NUM_MOTORS];

	PERIPHERAL_IO_ALL_t IO;

	//External IO panel control
	peripheral_exio_t PANEL;

	uint32_t oldTick;
	uint32_t deltaTick;


} PERIPHERAL_t;



/* Exported constants --------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
RD_RET RD_PERIPHERAL_INIT(PERIPHERAL_t* GPIO);
RD_RET RD_EXIO_INIT(peripheral_exio_t* exio);

RD_RET RD_PERIPHERAL_WRITE(PERIPHERAL_t* GPIO); //Drive IO need
RD_RET RD_PERIPHERAL_READ(PERIPHERAL_t* GPIO); //Read IO state what we need

RD_RET RD_EXIO_UPDATE(peripheral_exio_t* exio);
RD_RET RD_DPCA_UPDATE(PERIPHERAL_t* GPIO, PACKET_comm_t* PACKET);

RD_RET RD_MOT_DRIVE(PERIPHERAL_MOT_t* MOT, int32_t speed);
RD_RET RD_MOT_FORCE_DRIVE(PERIPHERAL_MOT_t* MOT, int32_t speed);

#endif /* INC_RD_PERIPHERAL_DPCB_H_ */
