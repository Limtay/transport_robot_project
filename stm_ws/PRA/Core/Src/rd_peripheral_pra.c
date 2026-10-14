/*
 * rd_peripheral_pra.c
 *
 *  Created on: Jan 27, 2026
 *      Author: abc01
 *
 *  PRA 페리페럴 추상화 계층 구현 (전원 GPIO+AUX / 자체 ADC / MCP23017 EXIO).
 */


/* Includes ------------------------------------------------------------------*/
#include <rd_peripheral_pra.h>
#include <string.h>
#include <math.h>

/* Exported includes ----------------------------------------------------------*/

/* Exported typedef -----------------------------------------------------------*/

/* Exported define ------------------------------------------------------------*/


/* Exported variables ---------------------------------------------------------*/
extern PERIPHERAL_t PRA_PERIPHERAL;
extern I2C_HandleTypeDef hi2c1;
extern ADC_HandleTypeDef hadc1;

/* Exported function prototypes -----------------------------------------------*/
RD_RET RD_PERIPHERAL_INIT(PERIPHERAL_t* GPIO);
RD_RET RD_EXIO_INIT(peripheral_exio_t* exio);

RD_RET RD_PERIPHERAL_WRITE(PERIPHERAL_t* GPIO); //Drive IO need
RD_RET RD_PERIPHERAL_READ(PERIPHERAL_t* GPIO); //Read IO state what we need

RD_RET RD_EXIO_UPDATE(peripheral_exio_t* exio);



/* Private user code ---------------------------------------------------------*/

/*==================== EXIO PVC ===================*/
RD_RET RD_EXIO_INIT(peripheral_exio_t* exio){
	//Port A:0~7, Port B:8:15
	HAL_GPIO_WritePin(exio_RESET_GPIO_Port, exio_RESET_Pin, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(exio_RESET_GPIO_Port, exio_RESET_Pin, GPIO_PIN_SET);

	// Output 설정
	EXIO_Set_OUTPUT(exio->hi2c, EXIO_LED1);
	EXIO_Set_OUTPUT(exio->hi2c, EXIO_LED2);
	EXIO_Set_OUTPUT(exio->hi2c, EXIO_LED3);

	// Input 설정
	EXIO_Set_INPUT(exio->hi2c, EXIO_SWA1);
	EXIO_Set_INPUT(exio->hi2c, EXIO_SWA2);


	return RET_OK;
}

RD_RET RD_EXIO_UPDATE(peripheral_exio_t* exio){

	//READ
	exio->SW1_state = EXIO_ReadPin(exio->hi2c, EXIO_SWA1);
	exio->SW2_state = EXIO_ReadPin(exio->hi2c, EXIO_SWA2);

	//WRITE
	EXIO_WritePin(exio->hi2c, EXIO_LED1, exio->LED1_state);
	EXIO_WritePin(exio->hi2c, EXIO_LED2, exio->LED2_state);
	EXIO_WritePin(exio->hi2c, EXIO_LED3, exio->BUZ_state);

	return RET_OK;
}

/*==================== ADC ===================*/

uint16_t adc_raw_buf[6];      // DMA가 무지성으로 값을 때려 박는 하드웨어 버퍼

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if(hadc->Instance == ADC1)
    {
        // 스냅샷
    	PRA_PERIPHERAL.adc_value[0] = (1.0f - ADC_ALPHA) * PRA_PERIPHERAL.adc_value[0] + ADC_ALPHA * adc_raw_buf[0];
    	PRA_PERIPHERAL.adc_value[1] = (1.0f - ADC_ALPHA) * PRA_PERIPHERAL.adc_value[1] + ADC_ALPHA * adc_raw_buf[1];
    	PRA_PERIPHERAL.adc_value[2] = (1.0f - ADC_ALPHA) * PRA_PERIPHERAL.adc_value[2] + ADC_ALPHA * adc_raw_buf[2];
    	PRA_PERIPHERAL.adc_value[3] = (1.0f - ADC_ALPHA) * PRA_PERIPHERAL.adc_value[3] + ADC_ALPHA * adc_raw_buf[3];
    	PRA_PERIPHERAL.adc_value[4] = (1.0f - ADC_ALPHA) * PRA_PERIPHERAL.adc_value[4] + ADC_ALPHA * adc_raw_buf[4];
    	PRA_PERIPHERAL.adc_value[5] = (1.0f - ADC_ALPHA) * PRA_PERIPHERAL.adc_value[5] + ADC_ALPHA * adc_raw_buf[5];
    }
}

float Convert_NTC_To_Celsius(uint16_t adc_value)
{
    // 오픈/쇼트 서킷 결선 오류 및 예외 처리
    if (adc_value >= 4095) {
        return -999.0f;
    }

    // 1. 현재 온도 상태에서의 NTC 저항(R_NTC) 유도 계산 (R_ref = 10k)
    float ntc_resistance = 10000.0f * ((4095.0f / (float)adc_value) - 1.0f);

    // 2. Steinhart-Hart Beta 공식을 이용한 절대 온도(Kelvin) 변환
    // 1/T = 1/T0 + (1/Beta) * ln(R/R0)
    float k_temperature = 1.0f / ( (1.0f / (25.0f + ABS_ZERO_TEMP)) +
                          (1.0f / NTC_BETA) * log(ntc_resistance / NTC_R25) );

    // 3. 절대 온도를 우리에게 익숙한 섭씨 온도(°C) 단위로 환산
    float c_temperature = k_temperature - ABS_ZERO_TEMP;

    return c_temperature;
}

float Convert_ADC_To_Voltage(uint16_t adc_value)
{
    // 오픈/쇼트 서킷 결선 오류 및 예외 처리
    if (adc_value >= 4095) {
        return -999.0f;
    }

    float v_voltage = adc_value * 0.0245f; //1:1 절연아날로그 연결, 30.4배, Vref 3.3v.  adc/4096 * 3.3 * 30.4

    return v_voltage;
}

float Convert_ADC_To_Current(uint16_t adc_value)
{
    // 오픈쇼트 서킷 결선 오류 및 예외 처리
    if (adc_value >= 4095) {
        return -999.0f;
    }

    float i_current = (adc_value - 2110) * 0.1465f; //1ㄷ1연결, 5.5mV/A, Vref 3.3v.  adc/4096 * 3.3 * 30.4

    return i_current;
}

RD_RET RD_ADC_INIT(void)
{
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_raw_buf, ADC_BUFFER_SIZE);

	return RET_OK;
}


RD_RET RD_UPDATE_ADC(PERIPHERAL_t* PERI)
{
	if (PERI == NULL) return RET_NOK;
	PERI->NTC[0] = Convert_NTC_To_Celsius(PERI->adc_value[2]);
	PERI->NTC[1] = Convert_NTC_To_Celsius(PERI->adc_value[1]);
	PERI->NTC[2] = Convert_NTC_To_Celsius(PERI->adc_value[0]);

	PERI->SENSOR_Voltage_IN  = Convert_ADC_To_Voltage(PERI->adc_value[5]);
	PERI->SENSOR_Voltage_OUT = Convert_ADC_To_Voltage(PERI->adc_value[4]);

	PERI->SENSOR_Current_BUS = Convert_ADC_To_Current(PERI->adc_value[3]);

	return RET_OK;
}


/*==================== Externel PVC ===================*/

/*==================== GENERAL ===================*/

RD_RET RD_PERIPHERAL_INIT(PERIPHERAL_t* GPIO)
{
	if (GPIO == NULL) return RET_NOK;

	/*============================== PIN INIT ==============================*/

	//INIT BOOT pin
	GPIO->IO.GPIO_RelayP.per_GPIOx 		= RP_EN_GPIO_Port;
	GPIO->IO.GPIO_RelayP.per_GPIO_Pin 	= RP_EN_Pin;

	GPIO->IO.GPIO_RelayN.per_GPIOx 		= RN_EN_GPIO_Port;
	GPIO->IO.GPIO_RelayN.per_GPIO_Pin 	= RN_EN_Pin;

	GPIO->IO.GPIO_SSP.per_GPIOx 		= PC_EN_GPIO_Port;
	GPIO->IO.GPIO_SSP.per_GPIO_Pin 		= PC_EN_Pin;

	GPIO->IO.GPIO_AUX1.per_GPIOx 		= AUX1_EN_GPIO_Port;
	GPIO->IO.GPIO_AUX1.per_GPIO_Pin 	= AUX1_EN_Pin;

	GPIO->IO.GPIO_AUX2.per_GPIOx 		= AUX2_EN_GPIO_Port;
	GPIO->IO.GPIO_AUX2.per_GPIO_Pin 	= AUX2_EN_Pin;

	/*============================== STATE INIT ==============================*/

	//INIT SOL state
	GPIO->EN_RelayP 	= 0;
	GPIO->EN_RelayN 	= 0;
	GPIO->EN_SSP 		= 0;
	GPIO->EN_AUX1 		= 0;
	GPIO->EN_AUX2 		= 0;

	/*============================== PERI INIT ==============================*/
	//EXIO
	GPIO->EXIO.hi2c = &hi2c1;
	RD_EXIO_INIT(&(GPIO->EXIO));

	//ADC
	RD_ADC_INIT();


	return RET_OK;
}


RD_RET RD_PERIPHERAL_WRITE(PERIPHERAL_t* GPIO)
{
	if (GPIO == NULL) return RET_NOK;

	/* ==================== WRITE PIN ====================*/
	if (GPIO->EN_RelayP == 1)			HAL_GPIO_WritePin(GPIO->IO.GPIO_RelayP.per_GPIOx, GPIO->IO.GPIO_RelayP.per_GPIO_Pin, GPIO_PIN_SET);
	else if (GPIO->EN_RelayP == 0)		HAL_GPIO_WritePin(GPIO->IO.GPIO_RelayP.per_GPIOx, GPIO->IO.GPIO_RelayP.per_GPIO_Pin, GPIO_PIN_RESET);

	if (GPIO->EN_RelayN == 1)			HAL_GPIO_WritePin(GPIO->IO.GPIO_RelayN.per_GPIOx, GPIO->IO.GPIO_RelayN.per_GPIO_Pin, GPIO_PIN_SET);
	else if (GPIO->EN_RelayN == 0)		HAL_GPIO_WritePin(GPIO->IO.GPIO_RelayN.per_GPIOx, GPIO->IO.GPIO_RelayN.per_GPIO_Pin, GPIO_PIN_RESET);

	if (GPIO->EN_SSP == 1)			HAL_GPIO_WritePin(GPIO->IO.GPIO_SSP.per_GPIOx, GPIO->IO.GPIO_SSP.per_GPIO_Pin, GPIO_PIN_SET);
	else if (GPIO->EN_SSP == 0)		HAL_GPIO_WritePin(GPIO->IO.GPIO_SSP.per_GPIOx, GPIO->IO.GPIO_SSP.per_GPIO_Pin, GPIO_PIN_RESET);

	if (GPIO->EN_AUX1 == 1)			HAL_GPIO_WritePin(GPIO->IO.GPIO_AUX1.per_GPIOx, GPIO->IO.GPIO_AUX1.per_GPIO_Pin, GPIO_PIN_SET);
	else if (GPIO->EN_AUX1 == 0)		HAL_GPIO_WritePin(GPIO->IO.GPIO_AUX1.per_GPIOx, GPIO->IO.GPIO_AUX1.per_GPIO_Pin, GPIO_PIN_RESET);

	if (GPIO->EN_AUX2 == 1)			HAL_GPIO_WritePin(GPIO->IO.GPIO_AUX2.per_GPIOx, GPIO->IO.GPIO_AUX2.per_GPIO_Pin, GPIO_PIN_SET);
	else if (GPIO->EN_AUX2 == 0)		HAL_GPIO_WritePin(GPIO->IO.GPIO_AUX2.per_GPIOx, GPIO->IO.GPIO_AUX2.per_GPIO_Pin, GPIO_PIN_RESET);
	return RET_OK;
}

RD_RET RD_PERIPHERAL_READ(PERIPHERAL_t* GPIO)
{
	if (GPIO == NULL) return RET_NOK;
	/* EXIO(MCP23017) 갱신은 RD_TASK_PERIPHERAL 루프의 RD_EXIO_UPDATE 에서 처리 */
	/* CAN(BMS) 는 can1Task 에서 별도 처리 (Step8) */

	/* ==================== READ ADC ==================== */
	RD_UPDATE_ADC(GPIO);   /* adc_value → Vin/Vout/Ibus/NTC 변환 */


	return RET_OK;
}

/* ==================== READ ME ====================*/
// 0. RD_PERIPHERAL_INIT은 "실제 핀의 배치를 추상화하는 함수"임. 실제로 포트바뀌면 여기서 다 수정해야되고, 기본적으로 main.h에서 1차적으로 정의되있긴함.
// 1. RD_PERIPHERAL_WRITE는 실제 핀에 값을 write 하는곳.
// 2. RD_PERIPHERAL_READ는 실제 핀의 로직값이나 센싱값을 스트럭쳐에 갱신하는곳
