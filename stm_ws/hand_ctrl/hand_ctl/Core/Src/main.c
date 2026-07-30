/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"s

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "rd_common.h"
#include "rd_define.h"

#include "rd_uart.h"
#include "rd_lasf_sc.h"

#include "rd_paxini_sc.h"

#include "rd_aft150_sc.h"

#include "rd_map_dyn.h"
#include "rd_peripheral.h"
#include "rd_uart.h"

#include "esc_hw.h"
#include "ecat_slv.h"
#include "options.h"
#include "utypes.h"
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart6_tx;
DMA_HandleTypeDef hdma_usart6_rx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for lasfTask */
osThreadId_t lasfTaskHandle;
const osThreadAttr_t lasfTask_attributes = {
  .name = "lasfTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for rs485Task */
osThreadId_t rs485TaskHandle;
const osThreadAttr_t rs485Task_attributes = {
  .name = "rs485Task",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for tactileTask */
osThreadId_t tactileTaskHandle;
const osThreadAttr_t tactileTask_attributes = {
  .name = "tactileTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for soesTask */
osThreadId_t soesTaskHandle;
const osThreadAttr_t soesTask_attributes = {
  .name = "soesTask",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* USER CODE BEGIN PV */

/*==========EtherCAT SPI1번==========*/
uint32_t esc_id = 0;
uint32_t sm0_temp = 0;
uint32_t sm1_temp = 0;

static esc_cfg_t esc_config =
{
    .user_arg = NULL,
    .use_interrupt = 0,
    .watchdog_cnt = 500,
    .skip_default_initialization = false,

    .set_defaults_hook = NULL,
    .pre_state_change_hook = NULL,
    .post_state_change_hook = NULL,
    .application_hook = NULL,
    .safeoutput_override = NULL,

    .pre_object_download_hook = NULL,
    .post_object_download_hook = NULL,
    .pre_object_upload_hook = NULL,
    .post_object_upload_hook = NULL,

    .rxpdo_override = NULL,
    .txpdo_override = NULL,

    .esc_hw_interrupt_enable = NULL,
    .esc_hw_interrupt_disable = NULL,
    .esc_hw_eep_handler = NULL,
    .esc_check_dc_handler = NULL,
    .get_device_id = NULL
};


_Objects    Obj;
uint8_t dir_toggle;


/*==========LASF usart6번==========*/
UART_Ring_t LASF_uart6 = {
    .rx_buffer = {0},           // 배열 초기화
    .head = 0,
    .tail = 0,
    .huart = &huart6,        // 미리 선언된 UART_HandleTypeDef

	.temp_buffer = {0},
	.rx_new = 0,
	.last_rx_tick = 0
};
LASF_COMM_t LASF[LASF_NUM];


/*==========paxini spi1번==========*/
uint8_t my_pdata[16];
PAXINI_PACKET_t tactile[3];
uint8_t paxini_ID[3] = {2,3,4};

/*==========USART1 RS485 (Dynamixel)==========*/
UART_Ring_t DYN_uart1 = {
    .huart      = &huart1,
    .is_running = 0
};

RS485_t DYN_rs485 = {
    .uart_obj = &DYN_uart1
};
uint32_t Diff_tick;

DYN_Ctrl_t xc330[3];


/*==========AFT150 세팅==========*/
AFT150_DATA_t AFT150_data;

CAN_FilterTypeDef sFilterConfig;



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_CAN1_Init(void);
static void MX_SPI2_Init(void);
void StartDefaultTask(void *argument);
void Start_lasf_Task(void *argument);
void Startrs485(void *argument);
void Start_tectile_Task(void *argument);
void StartsoesTask(void *argument);

/* USER CODE BEGIN PFP */
//void Aidin_Sensor_Start(void);
//void Aidin_Sensor_Cali(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void cb_get_inputs()
{
    // write to slave TxPDO
    // dummy value, so that sawtooth value profile will be seen constantly changing by ecat master

	/*==========AFT150==========*/
	//RX
	AFT150_data.FT_Offset[0] = Obj.AFT150_Config.ft_offset_fx;
	AFT150_data.FT_Offset[1] = Obj.AFT150_Config.ft_offset_fy;
	AFT150_data.FT_Offset[2] = Obj.AFT150_Config.ft_offset_fz;
	AFT150_data.FT_Offset[3] = Obj.AFT150_Config.ft_offset_mx;
	AFT150_data.FT_Offset[4] = Obj.AFT150_Config.ft_offset_my;
	AFT150_data.FT_Offset[5] = Obj.AFT150_Config.ft_offset_mz;
	//TX
	Obj.AFT150_T.ft_fx = AFT150_data.FT_Low[0] - AFT150_data.FT_Offset[0]; //offset
	Obj.AFT150_T.ft_fy = AFT150_data.FT_Low[1] - AFT150_data.FT_Offset[1];
	Obj.AFT150_T.ft_fz = AFT150_data.FT_Low[2] - AFT150_data.FT_Offset[2];
	Obj.AFT150_T.ft_mx = AFT150_data.FT_Low[3] - AFT150_data.FT_Offset[3];
	Obj.AFT150_T.ft_my = AFT150_data.FT_Low[4] - AFT150_data.FT_Offset[4];
	Obj.AFT150_T.ft_mz = AFT150_data.FT_Low[5] - AFT150_data.FT_Offset[5];

	/*==========LASF==========*/
	//SDO
	LASF[0].LASF.CTL_MODE 					= (uint16_t)Obj.LASF_Config_1.lasf_1_operating_mode;
	LASF[0].LASF.FT_OFFSET 					= Obj.LASF_Config_1.lasf_1_offset_force;
	//RX
	LASF[0].LASF.CTL_POS 					= Obj.LASF_1_R.lasf_1_target_position;
	LASF[0].LASF.CTL_FORCE 					= Obj.LASF_1_R.lasf_1_target_force;
	LASF[0].LASF.CTL_SPEED 					= Obj.LASF_1_R.lasf_1_target_speed;
	//TX
	Obj.LASF_1_T.lasf_1_actual_position 	= LASF[0].LASF.ACT_POS;
	Obj.LASF_1_T.lasf_1_actual_current 		= LASF[0].LASF.CURRENT;
	Obj.LASF_1_T.lasf_1_actual_force 		= LASF[0].LASF.FT_VAL;
	Obj.LASF_1_T.lasf_1_temperature 		= LASF[0].LASF.TEMP;
	Obj.LASF_1_T.lasf_1_error_code 			= LASF[0].LASF.Error_C;
	Obj.LASF_1_T.lasf_1_force_adc 			= LASF[0].LASF.FT_ADC;
	Obj.LASF_1_T.lasf_1_target_position 	= LASF[0].LASF.TAR_POS;

	//SDO
	LASF[1].LASF.CTL_MODE 					= (uint16_t)Obj.LASF_Config_2.lasf_2_operating_mode;
	LASF[1].LASF.FT_OFFSET 					= Obj.LASF_Config_2.lasf_2_offset_force;
	//RX
	LASF[1].LASF.CTL_POS 					= Obj.LASF_2_R.lasf_2_target_position;
	LASF[1].LASF.CTL_FORCE 					= Obj.LASF_2_R.lasf_2_target_force;
	LASF[1].LASF.CTL_SPEED 					= Obj.LASF_2_R.lasf_2_target_speed;
	//TX
	Obj.LASF_2_T.lasf_2_actual_position 	= LASF[1].LASF.ACT_POS;
	Obj.LASF_2_T.lasf_2_actual_current 		= LASF[1].LASF.CURRENT;
	Obj.LASF_2_T.lasf_2_actual_force 		= LASF[1].LASF.FT_VAL;
	Obj.LASF_2_T.lasf_2_temperature 		= LASF[1].LASF.TEMP;
	Obj.LASF_2_T.lasf_2_error_code 			= LASF[1].LASF.Error_C;
	Obj.LASF_2_T.lasf_2_force_adc 			= LASF[1].LASF.FT_ADC;
	Obj.LASF_2_T.lasf_2_target_position 	= LASF[1].LASF.TAR_POS;

	//SDO
	LASF[2].LASF.CTL_MODE 					= (uint16_t)Obj.LASF_Config_3.lasf_3_operating_mode;
	LASF[2].LASF.FT_OFFSET 					= Obj.LASF_Config_3.lasf_3_offset_force;
	//RX
	LASF[2].LASF.CTL_POS 					= Obj.LASF_3_R.lasf_3_target_position;
	LASF[2].LASF.CTL_FORCE 					= Obj.LASF_3_R.lasf_3_target_force;
	LASF[2].LASF.CTL_SPEED 					= Obj.LASF_3_R.lasf_3_target_speed;
	//TX
	Obj.LASF_3_T.lasf_3_actual_position 	= LASF[2].LASF.ACT_POS;
	Obj.LASF_3_T.lasf_3_actual_current 		= LASF[2].LASF.CURRENT;
	Obj.LASF_3_T.lasf_3_actual_force 		= LASF[2].LASF.FT_VAL;
	Obj.LASF_3_T.lasf_3_temperature 		= LASF[2].LASF.TEMP;
	Obj.LASF_3_T.lasf_3_error_code 			= LASF[2].LASF.Error_C;
	Obj.LASF_3_T.lasf_3_force_adc 			= LASF[2].LASF.FT_ADC;
	Obj.LASF_3_T.lasf_3_target_position 	= LASF[2].LASF.TAR_POS;


	//SDO
	LASF[3].LASF.CTL_MODE 					= (uint16_t)Obj.LASF_Config_4.lasf_4_operating_mode;
	LASF[3].LASF.FT_OFFSET 					= Obj.LASF_Config_4.lasf_4_offset_force;
	//RX
	LASF[3].LASF.CTL_POS 					= Obj.LASF_4_R.lasf_4_target_position;
	LASF[3].LASF.CTL_FORCE 					= Obj.LASF_4_R.lasf_4_target_force;
	LASF[3].LASF.CTL_SPEED 					= Obj.LASF_4_R.lasf_4_target_speed;
	//TX
	Obj.LASF_4_T.lasf_4_actual_position 	= LASF[3].LASF.ACT_POS;
	Obj.LASF_4_T.lasf_4_actual_current 		= LASF[3].LASF.CURRENT;
	Obj.LASF_4_T.lasf_4_actual_force 		= LASF[3].LASF.FT_VAL;
	Obj.LASF_4_T.lasf_4_temperature 		= LASF[3].LASF.TEMP;
	Obj.LASF_4_T.lasf_4_error_code 			= LASF[3].LASF.Error_C;
	Obj.LASF_4_T.lasf_4_force_adc 			= LASF[3].LASF.FT_ADC;
	Obj.LASF_4_T.lasf_4_target_position 	= LASF[3].LASF.TAR_POS;

	//SDO
	LASF[4].LASF.CTL_MODE 					= (uint16_t)Obj.LASF_Config_5.lasf_5_operating_mode;
	LASF[4].LASF.FT_OFFSET 					= Obj.LASF_Config_5.lasf_5_offset_force;
	//RX
	LASF[4].LASF.CTL_POS 					= Obj.LASF_5_R.lasf_5_target_position;
	LASF[4].LASF.CTL_FORCE 					= Obj.LASF_5_R.lasf_5_target_force;
	LASF[4].LASF.CTL_SPEED 					= Obj.LASF_5_R.lasf_5_target_speed;
	//TX
	Obj.LASF_5_T.lasf_5_actual_position 	= LASF[4].LASF.ACT_POS;
	Obj.LASF_5_T.lasf_5_actual_current 		= LASF[4].LASF.CURRENT;
	Obj.LASF_5_T.lasf_5_actual_force 		= LASF[4].LASF.FT_VAL;
	Obj.LASF_5_T.lasf_5_temperature 		= LASF[4].LASF.TEMP;
	Obj.LASF_5_T.lasf_5_error_code 			= LASF[4].LASF.Error_C;
	Obj.LASF_5_T.lasf_5_force_adc 			= LASF[4].LASF.FT_ADC;
	Obj.LASF_5_T.lasf_5_target_position 	= LASF[4].LASF.TAR_POS;

	//SDO
	LASF[5].LASF.CTL_MODE 					= (uint16_t)Obj.LASF_Config_6.lasf_6_operating_mode;
	LASF[5].LASF.FT_OFFSET 					= Obj.LASF_Config_6.lasf_6_offset_force;
	//RX
	LASF[5].LASF.CTL_POS 					= Obj.LASF_6_R.lasf_6_target_position;
	LASF[5].LASF.CTL_FORCE 					= Obj.LASF_6_R.lasf_6_target_force;
	LASF[5].LASF.CTL_SPEED 					= Obj.LASF_6_R.lasf_6_target_speed;
	//TX
	Obj.LASF_6_T.lasf_6_actual_position 	= LASF[5].LASF.ACT_POS;
	Obj.LASF_6_T.lasf_6_actual_current 		= LASF[5].LASF.CURRENT;
	Obj.LASF_6_T.lasf_6_actual_force 		= LASF[5].LASF.FT_VAL;
	Obj.LASF_6_T.lasf_6_temperature 		= LASF[5].LASF.TEMP;
	Obj.LASF_6_T.lasf_6_error_code 			= LASF[5].LASF.Error_C;
	Obj.LASF_6_T.lasf_6_force_adc 			= LASF[5].LASF.FT_ADC;
	Obj.LASF_6_T.lasf_6_target_position 	= LASF[5].LASF.TAR_POS;


	//SDO
	LASF[6].LASF.CTL_MODE 					= (uint16_t)Obj.LASF_Config_7.lasf_7_operating_mode;
	LASF[6].LASF.FT_OFFSET 					= Obj.LASF_Config_7.lasf_7_offset_force;
	//RX
	LASF[6].LASF.CTL_POS 					= Obj.LASF_7_R.lasf_7_target_position;
	LASF[6].LASF.CTL_FORCE 					= Obj.LASF_7_R.lasf_7_target_force;
	LASF[6].LASF.CTL_SPEED 					= Obj.LASF_7_R.lasf_7_target_speed;
	//TX
	Obj.LASF_7_T.lasf_7_actual_position 	= LASF[6].LASF.ACT_POS;
	Obj.LASF_7_T.lasf_7_actual_current 		= LASF[6].LASF.CURRENT;
	Obj.LASF_7_T.lasf_7_actual_force 		= LASF[6].LASF.FT_VAL;
	Obj.LASF_7_T.lasf_7_temperature 		= LASF[6].LASF.TEMP;
	Obj.LASF_7_T.lasf_7_error_code 			= LASF[6].LASF.Error_C;
	Obj.LASF_7_T.lasf_7_force_adc 			= LASF[6].LASF.FT_ADC;
	Obj.LASF_7_T.lasf_7_target_position 	= LASF[6].LASF.TAR_POS;

	//SDO
	LASF[7].LASF.CTL_MODE 					= (uint16_t)Obj.LASF_Config_8.lasf_8_operating_mode;
	LASF[7].LASF.FT_OFFSET 					= Obj.LASF_Config_8.lasf_8_offset_force;
	//RX
	LASF[7].LASF.CTL_POS 					= Obj.LASF_8_R.lasf_8_target_position;
	LASF[7].LASF.CTL_FORCE 					= Obj.LASF_8_R.lasf_8_target_force;
	LASF[7].LASF.CTL_SPEED 					= Obj.LASF_8_R.lasf_8_target_speed;
	//TX
	Obj.LASF_8_T.lasf_8_actual_position 	= LASF[7].LASF.ACT_POS;
	Obj.LASF_8_T.lasf_8_actual_current 		= LASF[7].LASF.CURRENT;
	Obj.LASF_8_T.lasf_8_actual_force 		= LASF[7].LASF.FT_VAL;
	Obj.LASF_8_T.lasf_8_temperature 		= LASF[7].LASF.TEMP;
	Obj.LASF_8_T.lasf_8_error_code 			= LASF[7].LASF.Error_C;
	Obj.LASF_8_T.lasf_8_force_adc 			= LASF[7].LASF.FT_ADC;
	Obj.LASF_8_T.lasf_8_target_position 	= LASF[7].LASF.TAR_POS;

	//SDO
	LASF[8].LASF.CTL_MODE 					= (uint16_t)Obj.LASF_Config_9.lasf_9_operating_mode;
	LASF[8].LASF.FT_OFFSET 					= Obj.LASF_Config_9.lasf_9_offset_force;
	//RX
	LASF[8].LASF.CTL_POS 					= Obj.LASF_9_R.lasf_9_target_position;
	LASF[8].LASF.CTL_FORCE 					= Obj.LASF_9_R.lasf_9_target_force;
	LASF[8].LASF.CTL_SPEED 					= Obj.LASF_9_R.lasf_9_target_speed;
	//TX
	Obj.LASF_9_T.lasf_9_actual_position 	= LASF[8].LASF.ACT_POS;
	Obj.LASF_9_T.lasf_9_actual_current 		= LASF[8].LASF.CURRENT;
	Obj.LASF_9_T.lasf_9_actual_force 		= LASF[8].LASF.FT_VAL;
	Obj.LASF_9_T.lasf_9_temperature 		= LASF[8].LASF.TEMP;
	Obj.LASF_9_T.lasf_9_error_code 			= LASF[8].LASF.Error_C;
	Obj.LASF_9_T.lasf_9_force_adc 			= LASF[8].LASF.FT_ADC;
	Obj.LASF_9_T.lasf_9_target_position 	= LASF[8].LASF.TAR_POS;
	/*==========Dynamixel==========*/
	//SDO
	if (xc330[0].ctl_enable == 0) xc330[0].ctl_mode = (DYN_MODE_e)Obj.Dynamixel_Config.dxl_1_operating_mode;
	//RX
	xc330[0].ram.cmd.goal_position				= Obj.Dynamixel_1_R.dxl_1_goal_position + 2048; //add offset
	xc330[0].ram.cmd.goal_current				= Obj.Dynamixel_1_R.dxl_1_goal_current;
	xc330[0].ram.cmd.goal_velocity 				= Obj.Dynamixel_1_R.dxl_1_goal_velocity;
	xc330[0].ctl_enable							= Obj.Dynamixel_1_R.dxl_1_torque_enable;
	//TX
	Obj.Dynamixel_1_T.dxl_1_present_position 	= xc330[0].ram.state.present_position - 2048; //restore offset
	Obj.Dynamixel_1_T.dxl_1_present_current 	= xc330[0].ram.state.present_current;
	Obj.Dynamixel_1_T.dxl_1_present_temperature = (int8_t)xc330[0].ram.state.present_temperature;
	Obj.Dynamixel_1_T.dxl_1_hardware_error 		= xc330[0].hardware_error;
	Obj.Dynamixel_1_T.dxl_1_present_velocity 	= xc330[0].ram.state.present_velocity;
	Obj.Dynamixel_1_T.dxl_1_moving 				= xc330[0].ram.state.moving;
	Obj.Dynamixel_1_T.dxl_1_moving_status 		= xc330[0].ram.state.moving_status;

	//SDO
	if (xc330[1].ctl_enable == 0) xc330[1].ctl_mode = (DYN_MODE_e)Obj.Dynamixel_Config.dxl_2_operating_mode;
	//RX
	xc330[1].ram.cmd.goal_position				= Obj.Dynamixel_2_R.dxl_2_goal_position + 2048;
	xc330[1].ram.cmd.goal_current				= Obj.Dynamixel_2_R.dxl_2_goal_current;
	xc330[1].ram.cmd.goal_velocity 				= Obj.Dynamixel_2_R.dxl_2_goal_velocity;
	xc330[1].ctl_enable							= Obj.Dynamixel_2_R.dxl_2_torque_enable;
	//TX
	Obj.Dynamixel_2_T.dxl_2_present_position 	= xc330[1].ram.state.present_position - 2048;
	Obj.Dynamixel_2_T.dxl_2_present_current 	= xc330[1].ram.state.present_current;
	Obj.Dynamixel_2_T.dxl_2_present_temperature = (int8_t)xc330[1].ram.state.present_temperature;
	Obj.Dynamixel_2_T.dxl_2_hardware_error 		= xc330[1].hardware_error;
	Obj.Dynamixel_2_T.dxl_2_present_velocity 	= xc330[1].ram.state.present_velocity;
	Obj.Dynamixel_2_T.dxl_2_moving 				= xc330[1].ram.state.moving;
	Obj.Dynamixel_2_T.dxl_2_moving_status 		= xc330[1].ram.state.moving_status;

	//SDO
	if (xc330[2].ctl_enable == 0) xc330[2].ctl_mode = (DYN_MODE_e)Obj.Dynamixel_Config.dxl_3_operating_mode;
	//RX
	xc330[2].ram.cmd.goal_position				= Obj.Dynamixel_3_R.dxl_3_goal_position + 2048;
	xc330[2].ram.cmd.goal_current				= Obj.Dynamixel_3_R.dxl_3_goal_current;
	xc330[2].ram.cmd.goal_velocity 				= Obj.Dynamixel_3_R.dxl_3_goal_velocity;
	xc330[2].ctl_enable							= Obj.Dynamixel_3_R.dxl_3_torque_enable;
	//TX
	Obj.Dynamixel_3_T.dxl_3_present_position 	= xc330[2].ram.state.present_position - 2048;
	Obj.Dynamixel_3_T.dxl_3_present_current 	= xc330[2].ram.state.present_current;
	Obj.Dynamixel_3_T.dxl_3_present_temperature = (int8_t)xc330[2].ram.state.present_temperature;
	Obj.Dynamixel_3_T.dxl_3_hardware_error 		= xc330[2].hardware_error;
	Obj.Dynamixel_3_T.dxl_3_present_velocity 	= xc330[2].ram.state.present_velocity;
	Obj.Dynamixel_3_T.dxl_3_moving 				= xc330[2].ram.state.moving;
	Obj.Dynamixel_3_T.dxl_3_moving_status 		= xc330[2].ram.state.moving_status;

	/*==========PAXINI_Tactile==========*/
	//SDO
	//TODO : add offset function
	//RX
	Obj.Tactile_T.tactile_1_fx					= tactile[0].GET_SUM.Fx;
	Obj.Tactile_T.tactile_1_fy					= tactile[0].GET_SUM.Fy;
	Obj.Tactile_T.tactile_1_fz					= tactile[0].GET_SUM.Fz;
	//TX - None

	//SDO
	//TODO : add offset function
	//RX
	Obj.Tactile_T.tactile_2_fx					= tactile[1].GET_SUM.Fx;
	Obj.Tactile_T.tactile_2_fy					= tactile[1].GET_SUM.Fy;
	Obj.Tactile_T.tactile_2_fz					= tactile[1].GET_SUM.Fz;
	//TX - None

	//SDO
	//TODO : add offset function
	//RX
	Obj.Tactile_T.tactile_3_fx					= tactile[2].GET_SUM.Fx;
	Obj.Tactile_T.tactile_3_fy					= tactile[2].GET_SUM.Fy;
	Obj.Tactile_T.tactile_3_fz					= tactile[2].GET_SUM.Fz;
	//TX - None

}

void cb_set_outputs()
{

}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  HAL_Delay(100);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USART6_UART_Init();
  MX_CAN1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

  /*=========PWR INIT==========*/
  HAL_GPIO_WritePin(EN_EXT_GPIO_Port, EN_EXT_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(EN_BUCK_GPIO_Port, EN_BUCK_Pin, GPIO_PIN_RESET);
  Paxini_Init(); //set spi mode

  HAL_Delay(500);

  HAL_GPIO_WritePin(EN_EXT_GPIO_Port, EN_EXT_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(EN_BUCK_GPIO_Port, EN_BUCK_Pin, GPIO_PIN_SET);
  HAL_Delay(500);

  /*==========LASF INIT==========*/
  /*
  if (RD_UART_INIT(&LASF_uart6) != RET_OK) Error_Handler();

  for (int i=0; i<LASF_NUM; i++){
	  RD_LASF_INIT(&LASF[i]);
	  LASF[i].LASF.ID = i+1;
	  LASF[i].RX.ID = i+1;
	  LASF[i].TX.ID = i+1;
  }
	*/

  /*==========PAXINI INIT==========*/
  //Paxini_Init();
  //HAL_Delay(100);
  //uint8_t tare_cmd = 1;
  // 1번 노드(Finger 1), 0x79 기능코드, 주소 3번에 1바이트(1) 쓰기
  //Paxini_Write(paxini_ID, 0x79, 3, 1, &tare_cmd);
  //HAL_Delay(100);

  /*==========DYN INIT==========*/

  /*==========AFT150 INIT==========*/

  //CAN FILTER SETTING
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;

  //sFilterConfig.FilterIdHigh = 0x230 << 5;
  //sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;

  //sFilterConfig.FilterMaskIdHigh = 0x7FE << 5;
  //sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;

  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = CAN_FILTER_ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;

  HAL_Delay(500);


  if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) {
      Error_Handler();
  }
  if (HAL_CAN_Start(&hcan1) != HAL_OK) {
      Error_Handler();
  }
  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
      Error_Handler();
  }

  Aidin_Sensor_Start(); //start sensor
  //Aidin_Sensor_Cali();	//baiasing sensor offset




  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of lasfTask */
  lasfTaskHandle = osThreadNew(Start_lasf_Task, NULL, &lasfTask_attributes);

  /* creation of rs485Task */
  rs485TaskHandle = osThreadNew(Startrs485, NULL, &rs485Task_attributes);

  /* creation of tactileTask */
  tactileTaskHandle = osThreadNew(Start_tectile_Task, NULL, &tactileTask_attributes);

  /* creation of soesTask */
  soesTaskHandle = osThreadNew(StartsoesTask, NULL, &soesTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 4;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_8TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 1000000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_8;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 921600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_8;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_G_Pin|LED_R_Pin|EN_EXT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, EN_BUCK_Pin|RS485_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SPI1_RST_Pin|SPI1_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SPI2_CS3_Pin|SPI2_CS2_Pin|SPI2_CS1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_G_Pin LED_R_Pin EN_EXT_Pin */
  GPIO_InitStruct.Pin = LED_G_Pin|LED_R_Pin|EN_EXT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PG_BUCK_Pin */
  GPIO_InitStruct.Pin = PG_BUCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PG_BUCK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_BUCK_Pin RS485_DIR_Pin */
  GPIO_InitStruct.Pin = EN_BUCK_Pin|RS485_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_RST_Pin SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_RST_Pin|SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI2_CS3_Pin SPI2_CS2_Pin SPI2_CS1_Pin */
  GPIO_InitStruct.Pin = SPI2_CS3_Pin|SPI2_CS2_Pin|SPI2_CS1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
	HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
    osDelay(200);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Start_lasf_Task */
/**
* @brief Function implementing the lasfTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_lasf_Task */
void Start_lasf_Task(void *argument)
{
  /* USER CODE BEGIN Start_lasf_Task */
  if (RD_UART_INIT(&LASF_uart6) != RET_OK) Error_Handler();

  for (int i=0; i<LASF_NUM; i++){
	  RD_LASF_INIT(&LASF[i]);
	  LASF[i].LASF.ID = i+1;
	  LASF[i].RX.ID = i+1;
	  LASF[i].TX.ID = i+1;
  }
  /* Infinite loop */
  for(;;)
  {
	  for (int i=0; i<LASF_NUM; i++){
		  RD_LASF_WRITE(&LASF_uart6, &LASF[i]);
		  RD_RET comm_state = RET_WAIT;
		  uint32_t comm_cnt = 0;

		  while (comm_state != RET_OK && comm_cnt <= 1){
			  osDelay(2);
			  comm_state = RD_LASF_READ(&LASF_uart6, &LASF[i]);

			  /*
		  	  if (comm_state == RET_OK){
			  HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
		  	  }
		  	  else if(comm_state == RET_NOK) HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET); //이거 나중에 sys_state를 default task에서 관리하면서 인디케이팅 하는게 맞을듯
		  	  */
		  	  comm_cnt++;
		  }
		  //RX
		  LASF[i].TX.SET_MODE = LASF[i].LASF.CTL_MODE;
		  LASF[i].TX.CTL_POS = LASF[i].LASF.CTL_POS;
		  LASF[i].TX.CTL_FORCE=LASF[i].LASF.CTL_FORCE+LASF[i].LASF.FT_OFFSET;
		  LASF[i].TX.CTL_SPEED=LASF[i].LASF.CTL_SPEED;

		  //TX
		  LASF[i].LASF.ACT_POS = LASF[i].RX.ACT_POS;
		  LASF[i].LASF.CURRENT = LASF[i].RX.CURRENT;
		  LASF[i].LASF.FT_VAL = LASF[i].RX.FT_VAL-LASF[i].LASF.FT_OFFSET;
		  LASF[i].LASF.TEMP = LASF[i].RX.TEMP;
		  LASF[i].LASF.Error_C = LASF[i].RX.Error_C;
		  LASF[i].LASF.FT_ADC = LASF[i].RX.FT_ADC;
		  LASF[i].LASF.TAR_POS = LASF[i].RX.TAR_POS;
	  }
	HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);

	osDelay(1);
  }
  /* USER CODE END Start_lasf_Task */
}

/* USER CODE BEGIN Header_Startrs485 */
/**
* @brief Function implementing the rs485Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Startrs485 */
void Startrs485(void *argument)
{
  /* USER CODE BEGIN Startrs485 */
  static const uint8_t DYN_IDS[3] = {2, 3, 4};

  /* ── 초기화 ──────────────────────────────────*/
  //osDelay(500);
  if (RD_RS485_INIT(&DYN_rs485) != RET_OK) Error_Handler();

  for (int i = 0; i < 3; i++) { //edit
	  if (RD_DYN_INIT(&xc330[i], DYN_IDS[i]) != RET_OK) Error_Handler();
	  for (int j = 0; j < 3; j++)
		  if (RD_DYN_INIT_SET(&DYN_rs485, &xc330[i]) != RET_WAIT) break;
  }

  /********** Simple Example ********/
  /*
  for (int j = 0; j < 3; j++){ //try repeat
	for (int i = 0; i < 3; i++) {
	xc330[i].inst = INST_WRITE;  			      // Instruction set
	xc330[i].addr.start  = DYN_ADDR_GOAL_CURRENT;  // Start Address set
	xc330[i].addr.size   = DYN_SIZE_GOAL_CURRENT;  // Data Length set
	xc330[i].ram.cmd.goal_current = 250;			  // Cmd data set. 2.69 [mA/U]
	RD_DYN_LOOP(&DYN_rs485, &xc330[i]);
	}
	osDelay(10);
  }
  */

  /**********************************/

  uint32_t tick_cnt = 0;
  for (;;)
  {
  /*====================================INF Loop BEGIN================================*/
	uint32_t start_tick = osKernelGetTickCount();
	for (int i = 0; i < 3; i++)
	{
	  if (++tick_cnt % 2 == 0){
		  RD_DYN_UPDATE_STATE(&xc330[i]);
		  RD_DYN_LOOP(&DYN_rs485, &xc330[i]);
		  osDelay(1);
		  RD_DYN_UPDATE_HWERROR(&xc330[i]);
		  RD_DYN_LOOP(&DYN_rs485, &xc330[i]);
		  osDelay(1);
	  } else {
		  RD_DYN_UPDATE_CMD(&xc330[i], xc330[i].ctl_mode);
		  RD_DYN_LOOP(&DYN_rs485, &xc330[i]);
		  osDelay(1);
		  if (xc330[i].mode != xc330[i].ctl_mode){ //모드변경
			  RD_DYN_OPERATE_ON(&xc330[i], xc330[i].ctl_mode);
			  RD_DYN_LOOP(&DYN_rs485, &xc330[i]);
			  osDelay(1);
		  }
		  if (xc330[i].enable != xc330[i].ctl_enable){
			  RD_DYN_TORQUE_ON(&xc330[i], xc330[i].ctl_enable);
			  RD_DYN_LOOP(&DYN_rs485, &xc330[i]);
			  osDelay(1);
		  }
	  }
	  //RD_DYN_LOOP(&DYN_rs485, &xc330[i]);
	}
	Diff_tick = osKernelGetTickCount() - start_tick;
  /*======================INF Loop END=======================*/
  }

  /* USER CODE END Startrs485 */
}

/* USER CODE BEGIN Header_Start_tectile_Task */
/**
* @brief Function implementing the tactileTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_tectile_Task */
void Start_tectile_Task(void *argument)
{
  /* USER CODE BEGIN Start_tectile_Task */
  //HAL_Delay(200);
  //Paxini_Init();
  //HAL_Delay(100);
  uint8_t tare_cmd = 1;
  for (int i=0; i<3; i++){
	  Paxini_Write(paxini_ID[i], 0x79, 3, 1, &tare_cmd);   // 1번 노드(Finger 1), 0x79 기능코드, 주소 3번에 1바이트(1) 쓰기
  }

  HAL_Delay(100);
  /* Infinite loop */
  for(;;)
  {
	  for (int i=0; i<3; i++){
		  RD_RET status = Paxini_Read(paxini_ID[i], 0x7b, 1008, 3, my_pdata);
		  if (status == RET_OK) {
			  if (my_pdata[0] == 0x01) {
				  tactile[i].GET_SUM.Fx = (float)my_pdata[1];
				  tactile[i].GET_SUM.Fy = (float)my_pdata[2];
				  tactile[i].GET_SUM.Fz = (float)my_pdata[3];
			  }
		  }
		  osDelay(1);
	  }

	  //RD_RET status = Paxini_Read(paxini_ID, 0x7b, 1038, 381, my_pdata);
	  /*
	  if (status == RET_OK) {
		  // check Status : respond 0x01
		  if (my_pdata[0] == 0x01) {
			  process_tactile_data(my_pdata, &tactile1);

		  }
	  }
		*/

    osDelay(10);

  }
  /* USER CODE END Start_tectile_Task */
}

/* USER CODE BEGIN Header_StartsoesTask */
/**
* @brief Function implementing the soesTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartsoesTask */
void StartsoesTask(void *argument)
{
  /* USER CODE BEGIN StartsoesTask */
  ecat_slv_init(&esc_config); //init ethercat slave

  esc_id = lan9252_read_32(0x0050); //connection check

  ESC_init(&esc_config);

  /* Infinite loop */
  for(;;)
  {
	ecat_slv();
	osDelay(1);
  }
  /* USER CODE END StartsoesTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
