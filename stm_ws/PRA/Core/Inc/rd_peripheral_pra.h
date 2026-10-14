/*
 * rd_peripheral_pra.h
 *
 *  Created on: Jan 27, 2026
 *      Author: abc01
 *
 *  PRA 페리페럴 추상화 계층 (전원 GPIO+AUX / 자체 ADC / MCP23017 EXIO).
 *  DPC_B rd_peripheral_dpcb 기반 이식.
 */

#ifndef INC_RD_PERIPHERAL_PRA_H_
#define INC_RD_PERIPHERAL_PRA_H_



/* Private includes ----------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"
#include "rd_common.h"
#include "rd_define.h"
#include "MCP23017.h"
#include "rd_peripheral.h"


/* Exported macro ------------------------------------------------------------*/
// GPA 포트 (0~7)
#define EXIO_SWA1    0    // GPA0
#define EXIO_LED1    1    // GPA1
#define EXIO_SWA2    2    // GPA2
#define EXIO_LED2    3    // GPA3
#define EXIO_SWA3    4    // GPA4, DNP
#define EXIO_LED3    5    // GPA5 use active buz instead led

// GPB 포트 (8~15)
//#define EXIO_SW6A    10   // GPB2
//#define EXIO_SW5A    11   // GPB3
//#define EXIO_SW4A    12   // GPB4
//#define EXIO_SW3A    13   // GPB5
//#define EXIO_SW2     14   // GPB6
//#define EXIO_SW1     15   // GPB7

//ADC - NTC
#define NTC_BETA        3435.0f  // B25/85 기준 고유 상수 값
#define NTC_R25         10000.0f // 25도에서의 기준 저항값 (10k)
#define ABS_ZERO_TEMP   273.15f  // 절대 영도 구하기용 상수

#define ADC_ALPHA		0.1f		/* ADC측정값 LPF */

#define ADC_BUFFER_SIZE	6


/* Exported types ------------------------------------------------------------*/

typedef struct {
	I2C_HandleTypeDef *hi2c;			//23017 gpio expander i2c handler
	//input pin state define
	uint8_t SW1_state;					//SPST x2 		: 0 is off, 1 is on
	uint8_t SW2_state;

	//output pin state define
	uint8_t LED1_state;					//SPST LED x2	: 0 is off, 1 is on
	uint8_t LED2_state;
	uint8_t BUZ_state;					//active buz
} peripheral_exio_t;


typedef struct {
	GPIO_IO_t GPIO_SSP;
	GPIO_IO_t GPIO_RelayP;
	GPIO_IO_t GPIO_RelayN;

	GPIO_IO_t GPIO_AUX1;
	GPIO_IO_t GPIO_AUX2;
} PERIPHERAL_IO_ALL_t;


typedef struct {
	//==================== ADC data ====================
	volatile float SENSOR_Voltage_IN;
	volatile float SENSOR_Voltage_OUT;
	volatile float SENSOR_Current_BUS;
	volatile float NTC[3];
	volatile uint16_t adc_value[6]; // 내가 가져다 쓸 안전한 내부 레지스터 변수

	//==================== CONTROL ====================
	//Solenoid CTRL
	volatile uint8_t EN_SSP;			//enable solid state precharger
	volatile uint8_t EN_RelayP;			//enable positive-side relay
	volatile uint8_t EN_RelayN;			//enable negative-side relay

	volatile uint8_t EN_AUX1;
	volatile uint8_t EN_AUX2;

	//==================== Pin def ====================
	PERIPHERAL_IO_ALL_t IO;

	//==================== GPIO ====================
	peripheral_exio_t EXIO;

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

/* ADC */
RD_RET RD_ADC_INIT(void);
RD_RET RD_UPDATE_ADC(PERIPHERAL_t* PERI);
float  Convert_NTC_To_Celsius(uint16_t adc_value);
float  Convert_ADC_To_Voltage(uint16_t adc_value);
float  Convert_ADC_To_Current(uint16_t adc_value);

#endif /* INC_RD_PERIPHERAL_PRA_H_ */
