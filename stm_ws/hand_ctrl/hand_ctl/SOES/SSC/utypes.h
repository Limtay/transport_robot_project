#ifndef __UTYPES_H__
#define __UTYPES_H__

#include "cc.h"

/* Object dictionary storage */

typedef struct
{
   /* Identity */

   uint32_t serial;

   /* Inputs */

   struct
   {
      int8_t tactile_1_fx;
      int8_t tactile_1_fy;
      uint8_t tactile_1_fz;
      int8_t tactile_2_fx;
      int8_t tactile_2_fy;
      uint8_t tactile_2_fz;
      int8_t tactile_3_fx;
      int8_t tactile_3_fy;
      uint8_t tactile_3_fz;
   } Tactile_T;
   struct
   {
      int32_t dxl_1_present_position;
      int16_t dxl_1_present_current;
      int8_t dxl_1_present_temperature;
      uint8_t dxl_1_hardware_error;
      int32_t dxl_1_present_velocity;
      uint8_t dxl_1_moving;
      uint8_t dxl_1_moving_status;
   } Dynamixel_1_T;
   struct
   {
      int32_t dxl_2_present_position;
      int16_t dxl_2_present_current;
      int8_t dxl_2_present_temperature;
      uint8_t dxl_2_hardware_error;
      int32_t dxl_2_present_velocity;
      uint8_t dxl_2_moving;
      uint8_t dxl_2_moving_status;
   } Dynamixel_2_T;
   struct
   {
      int32_t dxl_3_present_position;
      int16_t dxl_3_present_current;
      int8_t dxl_3_present_temperature;
      uint8_t dxl_3_hardware_error;
      int32_t dxl_3_present_velocity;
      uint8_t dxl_3_moving;
      uint8_t dxl_3_moving_status;
   } Dynamixel_3_T;
   struct
   {
      int16_t lasf_1_actual_position;
      uint16_t lasf_1_actual_current;
      int16_t lasf_1_actual_force;
      int8_t lasf_1_temperature;
      uint8_t lasf_1_error_code;
      int16_t lasf_1_target_position;
      uint16_t lasf_1_force_adc;
   } LASF_1_T;
   struct
   {
      int16_t lasf_2_actual_position;
      uint16_t lasf_2_actual_current;
      int16_t lasf_2_actual_force;
      int8_t lasf_2_temperature;
      uint8_t lasf_2_error_code;
      int16_t lasf_2_target_position;
      uint16_t lasf_2_force_adc;
   } LASF_2_T;
   struct
   {
      int16_t lasf_3_actual_position;
      uint16_t lasf_3_actual_current;
      int16_t lasf_3_actual_force;
      int8_t lasf_3_temperature;
      uint8_t lasf_3_error_code;
      int16_t lasf_3_target_position;
      uint16_t lasf_3_force_adc;
   } LASF_3_T;
   struct
   {
      int16_t lasf_4_actual_position;
      uint16_t lasf_4_actual_current;
      int16_t lasf_4_actual_force;
      int8_t lasf_4_temperature;
      uint8_t lasf_4_error_code;
      int16_t lasf_4_target_position;
      uint16_t lasf_4_force_adc;
   } LASF_4_T;
   struct
   {
      int16_t lasf_5_actual_position;
      uint16_t lasf_5_actual_current;
      int16_t lasf_5_actual_force;
      int8_t lasf_5_temperature;
      uint8_t lasf_5_error_code;
      int16_t lasf_5_target_position;
      uint16_t lasf_5_force_adc;
   } LASF_5_T;
   struct
   {
      int16_t lasf_6_actual_position;
      uint16_t lasf_6_actual_current;
      int16_t lasf_6_actual_force;
      int8_t lasf_6_temperature;
      uint8_t lasf_6_error_code;
      int16_t lasf_6_target_position;
      uint16_t lasf_6_force_adc;
   } LASF_6_T;
   struct
   {
      int16_t lasf_7_actual_position;
      uint16_t lasf_7_actual_current;
      int16_t lasf_7_actual_force;
      int8_t lasf_7_temperature;
      uint8_t lasf_7_error_code;
      int16_t lasf_7_target_position;
      uint16_t lasf_7_force_adc;
   } LASF_7_T;
   struct
   {
      int16_t lasf_8_actual_position;
      uint16_t lasf_8_actual_current;
      int16_t lasf_8_actual_force;
      int8_t lasf_8_temperature;
      uint8_t lasf_8_error_code;
      int16_t lasf_8_target_position;
      uint16_t lasf_8_force_adc;
   } LASF_8_T;
   struct
   {
      int16_t lasf_9_actual_position;
      uint16_t lasf_9_actual_current;
      int16_t lasf_9_actual_force;
      int8_t lasf_9_temperature;
      uint8_t lasf_9_error_code;
      int16_t lasf_9_target_position;
      uint16_t lasf_9_force_adc;
   } LASF_9_T;
   struct
   {
      int16_t ft_fx;
      int16_t ft_fy;
      int16_t ft_fz;
      int16_t ft_mx;
      int16_t ft_my;
      int16_t ft_mz;
   } AFT150_T;

   /* Outputs */

   struct
   {
      int32_t dxl_1_goal_position;
      int16_t dxl_1_goal_current;
      int32_t dxl_1_goal_velocity;
      uint8_t dxl_1_torque_enable;
   } Dynamixel_1_R;
   struct
   {
      int32_t dxl_2_goal_position;
      int16_t dxl_2_goal_current;
      int32_t dxl_2_goal_velocity;
      uint8_t dxl_2_torque_enable;
   } Dynamixel_2_R;
   struct
   {
      int32_t dxl_3_goal_position;
      int16_t dxl_3_goal_current;
      int32_t dxl_3_goal_velocity;
      uint8_t dxl_3_torque_enable;
   } Dynamixel_3_R;
   struct
   {
      int16_t lasf_1_target_position;
      int16_t lasf_1_target_force;
      uint16_t lasf_1_target_speed;
   } LASF_1_R;
   struct
   {
      int16_t lasf_2_target_position;
      int16_t lasf_2_target_force;
      uint16_t lasf_2_target_speed;
   } LASF_2_R;
   struct
   {
      int16_t lasf_3_target_position;
      int16_t lasf_3_target_force;
      uint16_t lasf_3_target_speed;
   } LASF_3_R;
   struct
   {
      int16_t lasf_4_target_position;
      int16_t lasf_4_target_force;
      uint16_t lasf_4_target_speed;
   } LASF_4_R;
   struct
   {
      int16_t lasf_5_target_position;
      int16_t lasf_5_target_force;
      uint16_t lasf_5_target_speed;
   } LASF_5_R;
   struct
   {
      int16_t lasf_6_target_position;
      int16_t lasf_6_target_force;
      uint16_t lasf_6_target_speed;
   } LASF_6_R;
   struct
   {
      int16_t lasf_7_target_position;
      int16_t lasf_7_target_force;
      uint16_t lasf_7_target_speed;
   } LASF_7_R;
   struct
   {
      int16_t lasf_8_target_position;
      int16_t lasf_8_target_force;
      uint16_t lasf_8_target_speed;
   } LASF_8_R;
   struct
   {
      int16_t lasf_9_target_position;
      int16_t lasf_9_target_force;
      uint16_t lasf_9_target_speed;
   } LASF_9_R;
   struct
   {
      uint8_t led_1_red;
      uint8_t led_1_green;
      uint8_t led_1_blue;
      uint8_t led_2_red;
      uint8_t led_2_green;
      uint8_t led_2_blue;
      uint8_t led_3_red;
      uint8_t led_3_green;
      uint8_t led_3_blue;
   } INDICATOR_R;

   /* Parameters */

   struct
   {
      uint8_t dxl_1_operating_mode;
      uint8_t dxl_2_operating_mode;
      uint8_t dxl_3_operating_mode;
   } Dynamixel_Config;
   struct
   {
      int8_t tactile_1_offset_fx;
      int8_t tactile_1_offset_fy;
      int8_t tactile_1_offset_fz;
      int8_t tactile_2_offset_fx;
      int8_t tactile_2_offset_fy;
      int8_t tactile_2_offset_fz;
      int8_t tactile_3_offset_fx;
      int8_t tactile_3_offset_fy;
      int8_t tactile_3_offset_fz;
   } Tactile_Config;
   struct
   {
      int16_t ft_offset_fx;
      int16_t ft_offset_fy;
      int16_t ft_offset_fz;
      int16_t ft_offset_mx;
      int16_t ft_offset_my;
      int16_t ft_offset_mz;
   } AFT150_Config;
   struct
   {
      uint8_t lasf_1_operating_mode;
      uint16_t lasf_1_stroke_upper_limit;
      uint16_t lasf_1_stroke_lower_limit;
      int16_t lasf_1_offset_force;
   } LASF_Config_1;
   struct
   {
      uint8_t lasf_2_operating_mode;
      uint16_t lasf_2_stroke_upper_limit;
      uint16_t lasf_2_stroke_lower_limit;
      int16_t lasf_2_offset_force;
   } LASF_Config_2;
   struct
   {
      uint8_t lasf_3_operating_mode;
      uint16_t lasf_3_stroke_upper_limit;
      uint16_t lasf_3_stroke_lower_limit;
      int16_t lasf_3_offset_force;
   } LASF_Config_3;
   struct
   {
      uint8_t lasf_4_operating_mode;
      uint16_t lasf_4_stroke_upper_limit;
      uint16_t lasf_4_stroke_lower_limit;
      int16_t lasf_4_offset_force;
   } LASF_Config_4;
   struct
   {
      uint8_t lasf_5_operating_mode;
      uint16_t lasf_5_stroke_upper_limit;
      uint16_t lasf_5_stroke_lower_limit;
      int16_t lasf_5_offset_force;
   } LASF_Config_5;
   struct
   {
      uint8_t lasf_6_operating_mode;
      uint16_t lasf_6_stroke_upper_limit;
      uint16_t lasf_6_stroke_lower_limit;
      int16_t lasf_6_offset_force;
   } LASF_Config_6;
   struct
   {
      uint8_t lasf_7_operating_mode;
      uint16_t lasf_7_stroke_upper_limit;
      uint16_t lasf_7_stroke_lower_limit;
      int16_t lasf_7_offset_force;
   } LASF_Config_7;
   struct
   {
      uint8_t lasf_8_operating_mode;
      uint16_t lasf_8_stroke_upper_limit;
      uint16_t lasf_8_stroke_lower_limit;
      int16_t lasf_8_offset_force;
   } LASF_Config_8;
   struct
   {
      uint8_t lasf_9_operating_mode;
      uint16_t lasf_9_stroke_upper_limit;
      uint16_t lasf_9_stroke_lower_limit;
      int16_t lasf_9_offset_force;
   } LASF_Config_9;
} _Objects;

extern _Objects Obj;

#endif /* __UTYPES_H__ */
