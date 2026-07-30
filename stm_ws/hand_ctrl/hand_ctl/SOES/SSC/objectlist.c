#include "esc_coe.h"
#include "utypes.h"
#include <stddef.h>


static const char acName1000[] = "Device Type";
static const char acName1008[] = "Device Name";
static const char acName1009[] = "Hardware Version";
static const char acName100A[] = "Software Version";
static const char acName1018[] = "Identity Object";
static const char acName1018_00[] = "Max SubIndex";
static const char acName1018_01[] = "Vendor ID";
static const char acName1018_02[] = "Product Code";
static const char acName1018_03[] = "Revision Number";
static const char acName1018_04[] = "Serial Number";
static const char acName1600[] = "Dynamixel_1_R";
static const char acName1600_00[] = "Max SubIndex";
static const char acName1600_01[] = "dxl_1_goal_position";
static const char acName1600_02[] = "dxl_1_goal_current";
static const char acName1600_03[] = "dxl_1_goal_velocity";
static const char acName1600_04[] = "dxl_1_torque_enable";
static const char acName1601[] = "Dynamixel_2_R";
static const char acName1601_00[] = "Max SubIndex";
static const char acName1601_01[] = "dxl_2_goal_position";
static const char acName1601_02[] = "dxl_2_goal_current";
static const char acName1601_03[] = "dxl_2_goal_velocity";
static const char acName1601_04[] = "dxl_2_torque_enable";
static const char acName1602[] = "Dynamixel_3_R";
static const char acName1602_00[] = "Max SubIndex";
static const char acName1602_01[] = "dxl_3_goal_position";
static const char acName1602_02[] = "dxl_3_goal_current";
static const char acName1602_03[] = "dxl_3_goal_velocity";
static const char acName1602_04[] = "dxl_3_torque_enable";
static const char acName1603[] = "LASF_1_R";
static const char acName1603_00[] = "Max SubIndex";
static const char acName1603_01[] = "lasf_1_target_position";
static const char acName1603_02[] = "lasf_1_target_force";
static const char acName1603_03[] = "lasf_1_target_speed";
static const char acName1604[] = "LASF_2_R";
static const char acName1604_00[] = "Max SubIndex";
static const char acName1604_01[] = "lasf_2_target_position";
static const char acName1604_02[] = "lasf_2_target_force";
static const char acName1604_03[] = "lasf_2_target_speed";
static const char acName1605[] = "LASF_3_R";
static const char acName1605_00[] = "Max SubIndex";
static const char acName1605_01[] = "lasf_3_target_position";
static const char acName1605_02[] = "lasf_3_target_force";
static const char acName1605_03[] = "lasf_3_target_speed";
static const char acName1606[] = "LASF_4_R";
static const char acName1606_00[] = "Max SubIndex";
static const char acName1606_01[] = "lasf_4_target_position";
static const char acName1606_02[] = "lasf_4_target_force";
static const char acName1606_03[] = "lasf_4_target_speed";
static const char acName1607[] = "LASF_5_R";
static const char acName1607_00[] = "Max SubIndex";
static const char acName1607_01[] = "lasf_5_target_position";
static const char acName1607_02[] = "lasf_5_target_force";
static const char acName1607_03[] = "lasf_5_target_speed";
static const char acName1608[] = "LASF_6_R";
static const char acName1608_00[] = "Max SubIndex";
static const char acName1608_01[] = "lasf_6_target_position";
static const char acName1608_02[] = "lasf_6_target_force";
static const char acName1608_03[] = "lasf_6_target_speed";
static const char acName1609[] = "LASF_7_R";
static const char acName1609_00[] = "Max SubIndex";
static const char acName1609_01[] = "lasf_7_target_position";
static const char acName1609_02[] = "lasf_7_target_force";
static const char acName1609_03[] = "lasf_7_target_speed";
static const char acName160A[] = "LASF_8_R";
static const char acName160A_00[] = "Max SubIndex";
static const char acName160A_01[] = "lasf_8_target_position";
static const char acName160A_02[] = "lasf_8_target_force";
static const char acName160A_03[] = "lasf_8_target_speed";
static const char acName160B[] = "LASF_9_R";
static const char acName160B_00[] = "Max SubIndex";
static const char acName160B_01[] = "lasf_9_target_position";
static const char acName160B_02[] = "lasf_9_target_force";
static const char acName160B_03[] = "lasf_9_target_speed";
static const char acName160C[] = "INDICATOR_R";
static const char acName160C_00[] = "Max SubIndex";
static const char acName160C_01[] = "led_1_red";
static const char acName160C_02[] = "led_1_green";
static const char acName160C_03[] = "led_1_blue";
static const char acName160C_04[] = "led_2_red";
static const char acName160C_05[] = "led_2_green";
static const char acName160C_06[] = "led_2_blue";
static const char acName160C_07[] = "led_3_red";
static const char acName160C_08[] = "led_3_green";
static const char acName160C_09[] = "led_3_blue";
static const char acName1A00[] = "Tactile_T";
static const char acName1A00_00[] = "Max SubIndex";
static const char acName1A00_01[] = "tactile_1_fx";
static const char acName1A00_02[] = "tactile_1_fy";
static const char acName1A00_03[] = "tactile_1_fz";
static const char acName1A00_04[] = "tactile_2_fx";
static const char acName1A00_05[] = "tactile_2_fy";
static const char acName1A00_06[] = "tactile_2_fz";
static const char acName1A00_07[] = "tactile_3_fx";
static const char acName1A00_08[] = "tactile_3_fy";
static const char acName1A00_09[] = "tactile_3_fz";
static const char acName1A01[] = "Dynamixel_1_T";
static const char acName1A01_00[] = "Max SubIndex";
static const char acName1A01_01[] = "dxl_1_present_position";
static const char acName1A01_02[] = "dxl_1_present_current";
static const char acName1A01_03[] = "dxl_1_present_temperature";
static const char acName1A01_04[] = "dxl_1_hardware_error";
static const char acName1A01_05[] = "dxl_1_present_velocity";
static const char acName1A01_06[] = "dxl_1_moving";
static const char acName1A01_07[] = "dxl_1_moving_status";
static const char acName1A02[] = "Dynamixel_2_T";
static const char acName1A02_00[] = "Max SubIndex";
static const char acName1A02_01[] = "dxl_2_present_position";
static const char acName1A02_02[] = "dxl_2_present_current";
static const char acName1A02_03[] = "dxl_2_present_temperature";
static const char acName1A02_04[] = "dxl_2_hardware_error";
static const char acName1A02_05[] = "dxl_2_present_velocity";
static const char acName1A02_06[] = "dxl_2_moving";
static const char acName1A02_07[] = "dxl_2_moving_status";
static const char acName1A03[] = "Dynamixel_3_T";
static const char acName1A03_00[] = "Max SubIndex";
static const char acName1A03_01[] = "dxl_3_present_position";
static const char acName1A03_02[] = "dxl_3_present_current";
static const char acName1A03_03[] = "dxl_3_present_temperature";
static const char acName1A03_04[] = "dxl_3_hardware_error";
static const char acName1A03_05[] = "dxl_3_present_velocity";
static const char acName1A03_06[] = "dxl_3_moving";
static const char acName1A03_07[] = "dxl_3_moving_status";
static const char acName1A04[] = "LASF_1_T";
static const char acName1A04_00[] = "Max SubIndex";
static const char acName1A04_01[] = "lasf_1_actual_position";
static const char acName1A04_02[] = "lasf_1_actual_current";
static const char acName1A04_03[] = "lasf_1_actual_force";
static const char acName1A04_04[] = "lasf_1_temperature";
static const char acName1A04_05[] = "lasf_1_error_code";
static const char acName1A04_06[] = "lasf_1_target_position";
static const char acName1A04_07[] = "lasf_1_force_adc";
static const char acName1A05[] = "LASF_2_T";
static const char acName1A05_00[] = "Max SubIndex";
static const char acName1A05_01[] = "lasf_2_actual_position";
static const char acName1A05_02[] = "lasf_2_actual_current";
static const char acName1A05_03[] = "lasf_2_actual_force";
static const char acName1A05_04[] = "lasf_2_temperature";
static const char acName1A05_05[] = "lasf_2_error_code";
static const char acName1A05_06[] = "lasf_2_target_position";
static const char acName1A05_07[] = "lasf_2_force_adc";
static const char acName1A06[] = "LASF_3_T";
static const char acName1A06_00[] = "Max SubIndex";
static const char acName1A06_01[] = "lasf_3_actual_position";
static const char acName1A06_02[] = "lasf_3_actual_current";
static const char acName1A06_03[] = "lasf_3_actual_force";
static const char acName1A06_04[] = "lasf_3_temperature";
static const char acName1A06_05[] = "lasf_3_error_code";
static const char acName1A06_06[] = "lasf_3_target_position";
static const char acName1A06_07[] = "lasf_3_force_adc";
static const char acName1A07[] = "LASF_4_T";
static const char acName1A07_00[] = "Max SubIndex";
static const char acName1A07_01[] = "lasf_4_actual_position";
static const char acName1A07_02[] = "lasf_4_actual_current";
static const char acName1A07_03[] = "lasf_4_actual_force";
static const char acName1A07_04[] = "lasf_4_temperature";
static const char acName1A07_05[] = "lasf_4_error_code";
static const char acName1A07_06[] = "lasf_4_target_position";
static const char acName1A07_07[] = "lasf_4_force_adc";
static const char acName1A08[] = "LASF_5_T";
static const char acName1A08_00[] = "Max SubIndex";
static const char acName1A08_01[] = "lasf_5_actual_position";
static const char acName1A08_02[] = "lasf_5_actual_current";
static const char acName1A08_03[] = "lasf_5_actual_force";
static const char acName1A08_04[] = "lasf_5_temperature";
static const char acName1A08_05[] = "lasf_5_error_code";
static const char acName1A08_06[] = "lasf_5_target_position";
static const char acName1A08_07[] = "lasf_5_force_adc";
static const char acName1A09[] = "LASF_6_T";
static const char acName1A09_00[] = "Max SubIndex";
static const char acName1A09_01[] = "lasf_6_actual_position";
static const char acName1A09_02[] = "lasf_6_actual_current";
static const char acName1A09_03[] = "lasf_6_actual_force";
static const char acName1A09_04[] = "lasf_6_temperature";
static const char acName1A09_05[] = "lasf_6_error_code";
static const char acName1A09_06[] = "lasf_6_target_position";
static const char acName1A09_07[] = "lasf_6_force_adc";
static const char acName1A0A[] = "LASF_7_T";
static const char acName1A0A_00[] = "Max SubIndex";
static const char acName1A0A_01[] = "lasf_7_actual_position";
static const char acName1A0A_02[] = "lasf_7_actual_current";
static const char acName1A0A_03[] = "lasf_7_actual_force";
static const char acName1A0A_04[] = "lasf_7_temperature";
static const char acName1A0A_05[] = "lasf_7_error_code";
static const char acName1A0A_06[] = "lasf_7_target_position";
static const char acName1A0A_07[] = "lasf_7_force_adc";
static const char acName1A0B[] = "LASF_8_T";
static const char acName1A0B_00[] = "Max SubIndex";
static const char acName1A0B_01[] = "lasf_8_actual_position";
static const char acName1A0B_02[] = "lasf_8_actual_current";
static const char acName1A0B_03[] = "lasf_8_actual_force";
static const char acName1A0B_04[] = "lasf_8_temperature";
static const char acName1A0B_05[] = "lasf_8_error_code";
static const char acName1A0B_06[] = "lasf_8_target_position";
static const char acName1A0B_07[] = "lasf_8_force_adc";
static const char acName1A0C[] = "LASF_9_T";
static const char acName1A0C_00[] = "Max SubIndex";
static const char acName1A0C_01[] = "lasf_9_actual_position";
static const char acName1A0C_02[] = "lasf_9_actual_current";
static const char acName1A0C_03[] = "lasf_9_actual_force";
static const char acName1A0C_04[] = "lasf_9_temperature";
static const char acName1A0C_05[] = "lasf_9_error_code";
static const char acName1A0C_06[] = "lasf_9_target_position";
static const char acName1A0C_07[] = "lasf_9_force_adc";
static const char acName1A0D[] = "AFT150_T";
static const char acName1A0D_00[] = "Max SubIndex";
static const char acName1A0D_01[] = "ft_fx";
static const char acName1A0D_02[] = "ft_fy";
static const char acName1A0D_03[] = "ft_fz";
static const char acName1A0D_04[] = "ft_mx";
static const char acName1A0D_05[] = "ft_my";
static const char acName1A0D_06[] = "ft_mz";
static const char acName1C00[] = "Sync Manager Communication Type";
static const char acName1C00_00[] = "Max SubIndex";
static const char acName1C00_01[] = "Communications Type SM0";
static const char acName1C00_02[] = "Communications Type SM1";
static const char acName1C00_03[] = "Communications Type SM2";
static const char acName1C00_04[] = "Communications Type SM3";
static const char acName1C12[] = "Sync Manager 2 PDO Assignment";
static const char acName1C12_00[] = "Max SubIndex";
static const char acName1C12_01[] = "PDO Mapping";
static const char acName1C12_02[] = "PDO Mapping";
static const char acName1C12_03[] = "PDO Mapping";
static const char acName1C12_04[] = "PDO Mapping";
static const char acName1C12_05[] = "PDO Mapping";
static const char acName1C12_06[] = "PDO Mapping";
static const char acName1C12_07[] = "PDO Mapping";
static const char acName1C12_08[] = "PDO Mapping";
static const char acName1C12_09[] = "PDO Mapping";
static const char acName1C12_10[] = "PDO Mapping";
static const char acName1C12_11[] = "PDO Mapping";
static const char acName1C12_12[] = "PDO Mapping";
static const char acName1C12_13[] = "PDO Mapping";
static const char acName1C13[] = "Sync Manager 3 PDO Assignment";
static const char acName1C13_00[] = "Max SubIndex";
static const char acName1C13_01[] = "PDO Mapping";
static const char acName1C13_02[] = "PDO Mapping";
static const char acName1C13_03[] = "PDO Mapping";
static const char acName1C13_04[] = "PDO Mapping";
static const char acName1C13_05[] = "PDO Mapping";
static const char acName1C13_06[] = "PDO Mapping";
static const char acName1C13_07[] = "PDO Mapping";
static const char acName1C13_08[] = "PDO Mapping";
static const char acName1C13_09[] = "PDO Mapping";
static const char acName1C13_10[] = "PDO Mapping";
static const char acName1C13_11[] = "PDO Mapping";
static const char acName1C13_12[] = "PDO Mapping";
static const char acName1C13_13[] = "PDO Mapping";
static const char acName1C13_14[] = "PDO Mapping";
static const char acName2000[] = "Dynamixel_Config";
static const char acName2000_00[] = "Max SubIndex";
static const char acName2000_01[] = "dxl_1_operating_mode";
static const char acName2000_02[] = "dxl_2_operating_mode";
static const char acName2000_03[] = "dxl_3_operating_mode";
static const char acName2100[] = "Tactile_Config";
static const char acName2100_00[] = "Max SubIndex";
static const char acName2100_01[] = "tactile_1_offset_fx";
static const char acName2100_02[] = "tactile_1_offset_fy";
static const char acName2100_03[] = "tactile_1_offset_fz";
static const char acName2100_04[] = "tactile_2_offset_fx";
static const char acName2100_05[] = "tactile_2_offset_fy";
static const char acName2100_06[] = "tactile_2_offset_fz";
static const char acName2100_07[] = "tactile_3_offset_fx";
static const char acName2100_08[] = "tactile_3_offset_fy";
static const char acName2100_09[] = "tactile_3_offset_fz";
static const char acName2200[] = "AFT150_Config";
static const char acName2200_00[] = "Max SubIndex";
static const char acName2200_01[] = "ft_offset_fx";
static const char acName2200_02[] = "ft_offset_fy";
static const char acName2200_03[] = "ft_offset_fz";
static const char acName2200_04[] = "ft_offset_mx";
static const char acName2200_05[] = "ft_offset_my";
static const char acName2200_06[] = "ft_offset_mz";
static const char acName2300[] = "LASF_Config_1";
static const char acName2300_00[] = "Max SubIndex";
static const char acName2300_01[] = "lasf_1_operating_mode";
static const char acName2300_02[] = "lasf_1_stroke_upper_limit";
static const char acName2300_03[] = "lasf_1_stroke_lower_limit";
static const char acName2300_04[] = "lasf_1_offset_force";
static const char acName2301[] = "LASF_Config_2";
static const char acName2301_00[] = "Max SubIndex";
static const char acName2301_01[] = "lasf_2_operating_mode";
static const char acName2301_02[] = "lasf_2_stroke_upper_limit";
static const char acName2301_03[] = "lasf_2_stroke_lower_limit";
static const char acName2301_04[] = "lasf_2_offset_force";
static const char acName2302[] = "LASF_Config_3";
static const char acName2302_00[] = "Max SubIndex";
static const char acName2302_01[] = "lasf_3_operating_mode";
static const char acName2302_02[] = "lasf_3_stroke_upper_limit";
static const char acName2302_03[] = "lasf_3_stroke_lower_limit";
static const char acName2302_04[] = "lasf_3_offset_force";
static const char acName2303[] = "LASF_Config_4";
static const char acName2303_00[] = "Max SubIndex";
static const char acName2303_01[] = "lasf_4_operating_mode";
static const char acName2303_02[] = "lasf_4_stroke_upper_limit";
static const char acName2303_03[] = "lasf_4_stroke_lower_limit";
static const char acName2303_04[] = "lasf_4_offset_force";
static const char acName2304[] = "LASF_Config_5";
static const char acName2304_00[] = "Max SubIndex";
static const char acName2304_01[] = "lasf_5_operating_mode";
static const char acName2304_02[] = "lasf_5_stroke_upper_limit";
static const char acName2304_03[] = "lasf_5_stroke_lower_limit";
static const char acName2304_04[] = "lasf_5_offset_force";
static const char acName2305[] = "LASF_Config_6";
static const char acName2305_00[] = "Max SubIndex";
static const char acName2305_01[] = "lasf_6_operating_mode";
static const char acName2305_02[] = "lasf_6_stroke_upper_limit";
static const char acName2305_03[] = "lasf_6_stroke_lower_limit";
static const char acName2305_04[] = "lasf_6_offset_force";
static const char acName2306[] = "LASF_Config_7";
static const char acName2306_00[] = "Max SubIndex";
static const char acName2306_01[] = "lasf_7_operating_mode";
static const char acName2306_02[] = "lasf_7_stroke_upper_limit";
static const char acName2306_03[] = "lasf_7_stroke_lower_limit";
static const char acName2306_04[] = "lasf_7_offset_force";
static const char acName2307[] = "LASF_Config_8";
static const char acName2307_00[] = "Max SubIndex";
static const char acName2307_01[] = "lasf_8_operating_mode";
static const char acName2307_02[] = "lasf_8_stroke_upper_limit";
static const char acName2307_03[] = "lasf_8_stroke_lower_limit";
static const char acName2307_04[] = "lasf_8_offset_force";
static const char acName2308[] = "LASF_Config_9";
static const char acName2308_00[] = "Max SubIndex";
static const char acName2308_01[] = "lasf_9_operating_mode";
static const char acName2308_02[] = "lasf_9_stroke_upper_limit";
static const char acName2308_03[] = "lasf_9_stroke_lower_limit";
static const char acName2308_04[] = "lasf_9_offset_force";
static const char acName6000[] = "Tactile_T";
static const char acName6000_00[] = "Max SubIndex";
static const char acName6000_01[] = "tactile_1_fx";
static const char acName6000_02[] = "tactile_1_fy";
static const char acName6000_03[] = "tactile_1_fz";
static const char acName6000_04[] = "tactile_2_fx";
static const char acName6000_05[] = "tactile_2_fy";
static const char acName6000_06[] = "tactile_2_fz";
static const char acName6000_07[] = "tactile_3_fx";
static const char acName6000_08[] = "tactile_3_fy";
static const char acName6000_09[] = "tactile_3_fz";
static const char acName6100[] = "Dynamixel_1_T";
static const char acName6100_00[] = "Max SubIndex";
static const char acName6100_01[] = "dxl_1_present_position";
static const char acName6100_02[] = "dxl_1_present_current";
static const char acName6100_03[] = "dxl_1_present_temperature";
static const char acName6100_04[] = "dxl_1_hardware_error";
static const char acName6100_05[] = "dxl_1_present_velocity";
static const char acName6100_06[] = "dxl_1_moving";
static const char acName6100_07[] = "dxl_1_moving_status";
static const char acName6101[] = "Dynamixel_2_T";
static const char acName6101_00[] = "Max SubIndex";
static const char acName6101_01[] = "dxl_2_present_position";
static const char acName6101_02[] = "dxl_2_present_current";
static const char acName6101_03[] = "dxl_2_present_temperature";
static const char acName6101_04[] = "dxl_2_hardware_error";
static const char acName6101_05[] = "dxl_2_present_velocity";
static const char acName6101_06[] = "dxl_2_moving";
static const char acName6101_07[] = "dxl_2_moving_status";
static const char acName6102[] = "Dynamixel_3_T";
static const char acName6102_00[] = "Max SubIndex";
static const char acName6102_01[] = "dxl_3_present_position";
static const char acName6102_02[] = "dxl_3_present_current";
static const char acName6102_03[] = "dxl_3_present_temperature";
static const char acName6102_04[] = "dxl_3_hardware_error";
static const char acName6102_05[] = "dxl_3_present_velocity";
static const char acName6102_06[] = "dxl_3_moving";
static const char acName6102_07[] = "dxl_3_moving_status";
static const char acName6200[] = "LASF_1_T";
static const char acName6200_00[] = "Max SubIndex";
static const char acName6200_01[] = "lasf_1_actual_position";
static const char acName6200_02[] = "lasf_1_actual_current";
static const char acName6200_03[] = "lasf_1_actual_force";
static const char acName6200_04[] = "lasf_1_temperature";
static const char acName6200_05[] = "lasf_1_error_code";
static const char acName6200_06[] = "lasf_1_target_position";
static const char acName6200_07[] = "lasf_1_force_adc";
static const char acName6201[] = "LASF_2_T";
static const char acName6201_00[] = "Max SubIndex";
static const char acName6201_01[] = "lasf_2_actual_position";
static const char acName6201_02[] = "lasf_2_actual_current";
static const char acName6201_03[] = "lasf_2_actual_force";
static const char acName6201_04[] = "lasf_2_temperature";
static const char acName6201_05[] = "lasf_2_error_code";
static const char acName6201_06[] = "lasf_2_target_position";
static const char acName6201_07[] = "lasf_2_force_adc";
static const char acName6202[] = "LASF_3_T";
static const char acName6202_00[] = "Max SubIndex";
static const char acName6202_01[] = "lasf_3_actual_position";
static const char acName6202_02[] = "lasf_3_actual_current";
static const char acName6202_03[] = "lasf_3_actual_force";
static const char acName6202_04[] = "lasf_3_temperature";
static const char acName6202_05[] = "lasf_3_error_code";
static const char acName6202_06[] = "lasf_3_target_position";
static const char acName6202_07[] = "lasf_3_force_adc";
static const char acName6203[] = "LASF_4_T";
static const char acName6203_00[] = "Max SubIndex";
static const char acName6203_01[] = "lasf_4_actual_position";
static const char acName6203_02[] = "lasf_4_actual_current";
static const char acName6203_03[] = "lasf_4_actual_force";
static const char acName6203_04[] = "lasf_4_temperature";
static const char acName6203_05[] = "lasf_4_error_code";
static const char acName6203_06[] = "lasf_4_target_position";
static const char acName6203_07[] = "lasf_4_force_adc";
static const char acName6204[] = "LASF_5_T";
static const char acName6204_00[] = "Max SubIndex";
static const char acName6204_01[] = "lasf_5_actual_position";
static const char acName6204_02[] = "lasf_5_actual_current";
static const char acName6204_03[] = "lasf_5_actual_force";
static const char acName6204_04[] = "lasf_5_temperature";
static const char acName6204_05[] = "lasf_5_error_code";
static const char acName6204_06[] = "lasf_5_target_position";
static const char acName6204_07[] = "lasf_5_force_adc";
static const char acName6205[] = "LASF_6_T";
static const char acName6205_00[] = "Max SubIndex";
static const char acName6205_01[] = "lasf_6_actual_position";
static const char acName6205_02[] = "lasf_6_actual_current";
static const char acName6205_03[] = "lasf_6_actual_force";
static const char acName6205_04[] = "lasf_6_temperature";
static const char acName6205_05[] = "lasf_6_error_code";
static const char acName6205_06[] = "lasf_6_target_position";
static const char acName6205_07[] = "lasf_6_force_adc";
static const char acName6206[] = "LASF_7_T";
static const char acName6206_00[] = "Max SubIndex";
static const char acName6206_01[] = "lasf_7_actual_position";
static const char acName6206_02[] = "lasf_7_actual_current";
static const char acName6206_03[] = "lasf_7_actual_force";
static const char acName6206_04[] = "lasf_7_temperature";
static const char acName6206_05[] = "lasf_7_error_code";
static const char acName6206_06[] = "lasf_7_target_position";
static const char acName6206_07[] = "lasf_7_force_adc";
static const char acName6207[] = "LASF_8_T";
static const char acName6207_00[] = "Max SubIndex";
static const char acName6207_01[] = "lasf_8_actual_position";
static const char acName6207_02[] = "lasf_8_actual_current";
static const char acName6207_03[] = "lasf_8_actual_force";
static const char acName6207_04[] = "lasf_8_temperature";
static const char acName6207_05[] = "lasf_8_error_code";
static const char acName6207_06[] = "lasf_8_target_position";
static const char acName6207_07[] = "lasf_8_force_adc";
static const char acName6208[] = "LASF_9_T";
static const char acName6208_00[] = "Max SubIndex";
static const char acName6208_01[] = "lasf_9_actual_position";
static const char acName6208_02[] = "lasf_9_actual_current";
static const char acName6208_03[] = "lasf_9_actual_force";
static const char acName6208_04[] = "lasf_9_temperature";
static const char acName6208_05[] = "lasf_9_error_code";
static const char acName6208_06[] = "lasf_9_target_position";
static const char acName6208_07[] = "lasf_9_force_adc";
static const char acName6300[] = "AFT150_T";
static const char acName6300_00[] = "Max SubIndex";
static const char acName6300_01[] = "ft_fx";
static const char acName6300_02[] = "ft_fy";
static const char acName6300_03[] = "ft_fz";
static const char acName6300_04[] = "ft_mx";
static const char acName6300_05[] = "ft_my";
static const char acName6300_06[] = "ft_mz";
static const char acName7000[] = "Dynamixel_1_R";
static const char acName7000_00[] = "Max SubIndex";
static const char acName7000_01[] = "dxl_1_goal_position";
static const char acName7000_02[] = "dxl_1_goal_current";
static const char acName7000_03[] = "dxl_1_goal_velocity";
static const char acName7000_04[] = "dxl_1_torque_enable";
static const char acName7001[] = "Dynamixel_2_R";
static const char acName7001_00[] = "Max SubIndex";
static const char acName7001_01[] = "dxl_2_goal_position";
static const char acName7001_02[] = "dxl_2_goal_current";
static const char acName7001_03[] = "dxl_2_goal_velocity";
static const char acName7001_04[] = "dxl_2_torque_enable";
static const char acName7002[] = "Dynamixel_3_R";
static const char acName7002_00[] = "Max SubIndex";
static const char acName7002_01[] = "dxl_3_goal_position";
static const char acName7002_02[] = "dxl_3_goal_current";
static const char acName7002_03[] = "dxl_3_goal_velocity";
static const char acName7002_04[] = "dxl_3_torque_enable";
static const char acName7100[] = "LASF_1_R";
static const char acName7100_00[] = "Max SubIndex";
static const char acName7100_01[] = "lasf_1_target_position";
static const char acName7100_02[] = "lasf_1_target_force";
static const char acName7100_03[] = "lasf_1_target_speed";
static const char acName7101[] = "LASF_2_R";
static const char acName7101_00[] = "Max SubIndex";
static const char acName7101_01[] = "lasf_2_target_position";
static const char acName7101_02[] = "lasf_2_target_force";
static const char acName7101_03[] = "lasf_2_target_speed";
static const char acName7102[] = "LASF_3_R";
static const char acName7102_00[] = "Max SubIndex";
static const char acName7102_01[] = "lasf_3_target_position";
static const char acName7102_02[] = "lasf_3_target_force";
static const char acName7102_03[] = "lasf_3_target_speed";
static const char acName7103[] = "LASF_4_R";
static const char acName7103_00[] = "Max SubIndex";
static const char acName7103_01[] = "lasf_4_target_position";
static const char acName7103_02[] = "lasf_4_target_force";
static const char acName7103_03[] = "lasf_4_target_speed";
static const char acName7104[] = "LASF_5_R";
static const char acName7104_00[] = "Max SubIndex";
static const char acName7104_01[] = "lasf_5_target_position";
static const char acName7104_02[] = "lasf_5_target_force";
static const char acName7104_03[] = "lasf_5_target_speed";
static const char acName7105[] = "LASF_6_R";
static const char acName7105_00[] = "Max SubIndex";
static const char acName7105_01[] = "lasf_6_target_position";
static const char acName7105_02[] = "lasf_6_target_force";
static const char acName7105_03[] = "lasf_6_target_speed";
static const char acName7106[] = "LASF_7_R";
static const char acName7106_00[] = "Max SubIndex";
static const char acName7106_01[] = "lasf_7_target_position";
static const char acName7106_02[] = "lasf_7_target_force";
static const char acName7106_03[] = "lasf_7_target_speed";
static const char acName7107[] = "LASF_8_R";
static const char acName7107_00[] = "Max SubIndex";
static const char acName7107_01[] = "lasf_8_target_position";
static const char acName7107_02[] = "lasf_8_target_force";
static const char acName7107_03[] = "lasf_8_target_speed";
static const char acName7108[] = "LASF_9_R";
static const char acName7108_00[] = "Max SubIndex";
static const char acName7108_01[] = "lasf_9_target_position";
static const char acName7108_02[] = "lasf_9_target_force";
static const char acName7108_03[] = "lasf_9_target_speed";
static const char acName7200[] = "INDICATOR_R";
static const char acName7200_00[] = "Max SubIndex";
static const char acName7200_01[] = "led_1_red";
static const char acName7200_02[] = "led_1_green";
static const char acName7200_03[] = "led_1_blue";
static const char acName7200_04[] = "led_2_red";
static const char acName7200_05[] = "led_2_green";
static const char acName7200_06[] = "led_2_blue";
static const char acName7200_07[] = "led_3_red";
static const char acName7200_08[] = "led_3_green";
static const char acName7200_09[] = "led_3_blue";

const _objd SDO1000[] =
{
  {0x0, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1000, 5001, NULL},
};
const _objd SDO1008[] =
{
  {0x0, DTYPE_VISIBLE_STRING, 56, ATYPE_RO, acName1008, 0, "HD_HAND"},
};
const _objd SDO1009[] =
{
  {0x0, DTYPE_VISIBLE_STRING, 40, ATYPE_RO, acName1009, 0, "0.1.0"},
};
const _objd SDO100A[] =
{
  {0x0, DTYPE_VISIBLE_STRING, 40, ATYPE_RO, acName100A, 0, "0.1.3"},
};
const _objd SDO1018[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1018_00, 4, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1018_01, 4515, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1018_02, 1162674336, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1018_03, 1, NULL},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1018_04, 1, &Obj.serial},
};
const _objd SDO1600[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1600_00, 4, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1600_01, 0x70000120, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1600_02, 0x70000210, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1600_03, 0x70000320, NULL},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1600_04, 0x70000408, NULL},
};
const _objd SDO1601[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1601_00, 4, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1601_01, 0x70010120, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1601_02, 0x70010210, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1601_03, 0x70010320, NULL},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1601_04, 0x70010408, NULL},
};
const _objd SDO1602[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1602_00, 4, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1602_01, 0x70020120, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1602_02, 0x70020210, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1602_03, 0x70020320, NULL},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1602_04, 0x70020408, NULL},
};
const _objd SDO1603[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1603_00, 3, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1603_01, 0x71000110, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1603_02, 0x71000210, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1603_03, 0x71000310, NULL},
};
const _objd SDO1604[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1604_00, 3, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1604_01, 0x71010110, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1604_02, 0x71010210, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1604_03, 0x71010310, NULL},
};
const _objd SDO1605[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1605_00, 3, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1605_01, 0x71020110, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1605_02, 0x71020210, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1605_03, 0x71020310, NULL},
};
const _objd SDO1606[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1606_00, 3, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1606_01, 0x71030110, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1606_02, 0x71030210, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1606_03, 0x71030310, NULL},
};
const _objd SDO1607[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1607_00, 3, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1607_01, 0x71040110, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1607_02, 0x71040210, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1607_03, 0x71040310, NULL},
};
const _objd SDO1608[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1608_00, 3, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1608_01, 0x71050110, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1608_02, 0x71050210, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1608_03, 0x71050310, NULL},
};
const _objd SDO1609[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1609_00, 3, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1609_01, 0x71060110, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1609_02, 0x71060210, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1609_03, 0x71060310, NULL},
};
const _objd SDO160A[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName160A_00, 3, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName160A_01, 0x71070110, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName160A_02, 0x71070210, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName160A_03, 0x71070310, NULL},
};
const _objd SDO160B[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName160B_00, 3, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName160B_01, 0x71080110, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName160B_02, 0x71080210, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName160B_03, 0x71080310, NULL},
};
const _objd SDO160C[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName160C_00, 9, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName160C_01, 0x72000108, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName160C_02, 0x72000208, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName160C_03, 0x72000308, NULL},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName160C_04, 0x72000408, NULL},
  {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName160C_05, 0x72000508, NULL},
  {0x06, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName160C_06, 0x72000608, NULL},
  {0x07, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName160C_07, 0x72000708, NULL},
  {0x08, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName160C_08, 0x72000808, NULL},
  {0x09, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName160C_09, 0x72000908, NULL},
};
const _objd SDO1A00[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1A00_00, 9, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A00_01, 0x60000108, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A00_02, 0x60000208, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A00_03, 0x60000308, NULL},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A00_04, 0x60000408, NULL},
  {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A00_05, 0x60000508, NULL},
  {0x06, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A00_06, 0x60000608, NULL},
  {0x07, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A00_07, 0x60000708, NULL},
  {0x08, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A00_08, 0x60000808, NULL},
  {0x09, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A00_09, 0x60000908, NULL},
};
const _objd SDO1A01[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1A01_00, 7, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A01_01, 0x61000120, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A01_02, 0x61000210, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A01_03, 0x61000308, NULL},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A01_04, 0x61000408, NULL},
  {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A01_05, 0x61000520, NULL},
  {0x06, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A01_06, 0x61000608, NULL},
  {0x07, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A01_07, 0x61000708, NULL},
};
const _objd SDO1A02[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1A02_00, 7, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_01, 0x61010120, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_02, 0x61010210, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_03, 0x61010308, NULL},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_04, 0x61010408, NULL},
  {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_05, 0x61010520, NULL},
  {0x06, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_06, 0x61010608, NULL},
  {0x07, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A02_07, 0x61010708, NULL},
};
const _objd SDO1A03[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1A03_00, 7, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_01, 0x61020120, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_02, 0x61020210, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_03, 0x61020308, NULL},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_04, 0x61020408, NULL},
  {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_05, 0x61020520, NULL},
  {0x06, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_06, 0x61020608, NULL},
  {0x07, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A03_07, 0x61020708, NULL},
};
const _objd SDO1A04[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1A04_00, 7, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_01, 0x62000110, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_02, 0x62000210, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_03, 0x62000310, NULL},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_04, 0x62000408, NULL},
  {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_05, 0x62000508, NULL},
  {0x06, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_06, 0x62000610, NULL},
  {0x07, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A04_07, 0x62000710, NULL},
};
const _objd SDO1A05[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1A05_00, 7, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A05_01, 0x62010110, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A05_02, 0x62010210, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A05_03, 0x62010310, NULL},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A05_04, 0x62010408, NULL},
  {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A05_05, 0x62010508, NULL},
  {0x06, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A05_06, 0x62010610, NULL},
  {0x07, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A05_07, 0x62010710, NULL},
};
const _objd SDO1A06[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1A06_00, 7, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A06_01, 0x62020110, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A06_02, 0x62020210, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A06_03, 0x62020310, NULL},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A06_04, 0x62020408, NULL},
  {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A06_05, 0x62020508, NULL},
  {0x06, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A06_06, 0x62020610, NULL},
  {0x07, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A06_07, 0x62020710, NULL},
};
const _objd SDO1A07[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1A07_00, 7, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A07_01, 0x62030110, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A07_02, 0x62030210, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A07_03, 0x62030310, NULL},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A07_04, 0x62030408, NULL},
  {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A07_05, 0x62030508, NULL},
  {0x06, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A07_06, 0x62030610, NULL},
  {0x07, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A07_07, 0x62030710, NULL},
};
const _objd SDO1A08[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1A08_00, 7, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A08_01, 0x62040110, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A08_02, 0x62040210, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A08_03, 0x62040310, NULL},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A08_04, 0x62040408, NULL},
  {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A08_05, 0x62040508, NULL},
  {0x06, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A08_06, 0x62040610, NULL},
  {0x07, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A08_07, 0x62040710, NULL},
};
const _objd SDO1A09[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1A09_00, 7, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A09_01, 0x62050110, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A09_02, 0x62050210, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A09_03, 0x62050310, NULL},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A09_04, 0x62050408, NULL},
  {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A09_05, 0x62050508, NULL},
  {0x06, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A09_06, 0x62050610, NULL},
  {0x07, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A09_07, 0x62050710, NULL},
};
const _objd SDO1A0A[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1A0A_00, 7, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A0A_01, 0x62060110, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A0A_02, 0x62060210, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A0A_03, 0x62060310, NULL},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A0A_04, 0x62060408, NULL},
  {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A0A_05, 0x62060508, NULL},
  {0x06, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A0A_06, 0x62060610, NULL},
  {0x07, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A0A_07, 0x62060710, NULL},
};
const _objd SDO1A0B[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1A0B_00, 7, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A0B_01, 0x62070110, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A0B_02, 0x62070210, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A0B_03, 0x62070310, NULL},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A0B_04, 0x62070408, NULL},
  {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A0B_05, 0x62070508, NULL},
  {0x06, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A0B_06, 0x62070610, NULL},
  {0x07, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A0B_07, 0x62070710, NULL},
};
const _objd SDO1A0C[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1A0C_00, 7, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A0C_01, 0x62080110, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A0C_02, 0x62080210, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A0C_03, 0x62080310, NULL},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A0C_04, 0x62080408, NULL},
  {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A0C_05, 0x62080508, NULL},
  {0x06, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A0C_06, 0x62080610, NULL},
  {0x07, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A0C_07, 0x62080710, NULL},
};
const _objd SDO1A0D[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1A0D_00, 6, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A0D_01, 0x63000110, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A0D_02, 0x63000210, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A0D_03, 0x63000310, NULL},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A0D_04, 0x63000410, NULL},
  {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A0D_05, 0x63000510, NULL},
  {0x06, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A0D_06, 0x63000610, NULL},
};
const _objd SDO1C00[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C00_00, 4, NULL},
  {0x01, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C00_01, 1, NULL},
  {0x02, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C00_02, 2, NULL},
  {0x03, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C00_03, 3, NULL},
  {0x04, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C00_04, 4, NULL},
};
const _objd SDO1C12[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C12_00, 13, NULL},
  {0x01, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C12_01, 0x1600, NULL},
  {0x02, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C12_02, 0x1601, NULL},
  {0x03, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C12_03, 0x1602, NULL},
  {0x04, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C12_04, 0x1603, NULL},
  {0x05, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C12_05, 0x1604, NULL},
  {0x06, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C12_06, 0x1605, NULL},
  {0x07, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C12_07, 0x1606, NULL},
  {0x08, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C12_08, 0x1607, NULL},
  {0x09, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C12_09, 0x1608, NULL},
  {0x0a, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C12_10, 0x1609, NULL},
  {0x0b, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C12_11, 0x160A, NULL},
  {0x0c, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C12_12, 0x160B, NULL},
  {0x0d, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C12_13, 0x160C, NULL},
};
const _objd SDO1C13[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C13_00, 14, NULL},
  {0x01, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C13_01, 0x1A00, NULL},
  {0x02, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C13_02, 0x1A01, NULL},
  {0x03, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C13_03, 0x1A02, NULL},
  {0x04, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C13_04, 0x1A03, NULL},
  {0x05, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C13_05, 0x1A04, NULL},
  {0x06, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C13_06, 0x1A05, NULL},
  {0x07, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C13_07, 0x1A06, NULL},
  {0x08, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C13_08, 0x1A07, NULL},
  {0x09, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C13_09, 0x1A08, NULL},
  {0x0a, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C13_10, 0x1A09, NULL},
  {0x0b, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C13_11, 0x1A0A, NULL},
  {0x0c, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C13_12, 0x1A0B, NULL},
  {0x0d, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C13_13, 0x1A0C, NULL},
  {0x0e, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C13_14, 0x1A0D, NULL},
};
const _objd SDO2000[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName2000_00, 3, NULL},
  {0x01, DTYPE_UNSIGNED8, 8, ATYPE_RW, acName2000_01, 3, &Obj.Dynamixel_Config.dxl_1_operating_mode},
  {0x02, DTYPE_UNSIGNED8, 8, ATYPE_RW, acName2000_02, 3, &Obj.Dynamixel_Config.dxl_2_operating_mode},
  {0x03, DTYPE_UNSIGNED8, 8, ATYPE_RW, acName2000_03, 3, &Obj.Dynamixel_Config.dxl_3_operating_mode},
};
const _objd SDO2100[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName2100_00, 9, NULL},
  {0x01, DTYPE_INTEGER8, 8, ATYPE_RW, acName2100_01, 0, &Obj.Tactile_Config.tactile_1_offset_fx},
  {0x02, DTYPE_INTEGER8, 8, ATYPE_RW, acName2100_02, 0, &Obj.Tactile_Config.tactile_1_offset_fy},
  {0x03, DTYPE_INTEGER8, 8, ATYPE_RW, acName2100_03, 0, &Obj.Tactile_Config.tactile_1_offset_fz},
  {0x04, DTYPE_INTEGER8, 8, ATYPE_RW, acName2100_04, 0, &Obj.Tactile_Config.tactile_2_offset_fx},
  {0x05, DTYPE_INTEGER8, 8, ATYPE_RW, acName2100_05, 0, &Obj.Tactile_Config.tactile_2_offset_fy},
  {0x06, DTYPE_INTEGER8, 8, ATYPE_RW, acName2100_06, 0, &Obj.Tactile_Config.tactile_2_offset_fz},
  {0x07, DTYPE_INTEGER8, 8, ATYPE_RW, acName2100_07, 0, &Obj.Tactile_Config.tactile_3_offset_fx},
  {0x08, DTYPE_INTEGER8, 8, ATYPE_RW, acName2100_08, 0, &Obj.Tactile_Config.tactile_3_offset_fy},
  {0x09, DTYPE_INTEGER8, 8, ATYPE_RW, acName2100_09, 0, &Obj.Tactile_Config.tactile_3_offset_fz},
};
const _objd SDO2200[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName2200_00, 6, NULL},
  {0x01, DTYPE_INTEGER16, 16, ATYPE_RW, acName2200_01, 0, &Obj.AFT150_Config.ft_offset_fx},
  {0x02, DTYPE_INTEGER16, 16, ATYPE_RW, acName2200_02, 0, &Obj.AFT150_Config.ft_offset_fy},
  {0x03, DTYPE_INTEGER16, 16, ATYPE_RW, acName2200_03, 0, &Obj.AFT150_Config.ft_offset_fz},
  {0x04, DTYPE_INTEGER16, 16, ATYPE_RW, acName2200_04, 0, &Obj.AFT150_Config.ft_offset_mx},
  {0x05, DTYPE_INTEGER16, 16, ATYPE_RW, acName2200_05, 0, &Obj.AFT150_Config.ft_offset_my},
  {0x06, DTYPE_INTEGER16, 16, ATYPE_RW, acName2200_06, 0, &Obj.AFT150_Config.ft_offset_mz},
};
const _objd SDO2300[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName2300_00, 4, NULL},
  {0x01, DTYPE_UNSIGNED8, 8, ATYPE_RW, acName2300_01, 1, &Obj.LASF_Config_1.lasf_1_operating_mode},
  {0x02, DTYPE_UNSIGNED16, 16, ATYPE_RW, acName2300_02, 2000, &Obj.LASF_Config_1.lasf_1_stroke_upper_limit},
  {0x03, DTYPE_UNSIGNED16, 16, ATYPE_RW, acName2300_03, 0, &Obj.LASF_Config_1.lasf_1_stroke_lower_limit},
  {0x04, DTYPE_INTEGER16, 16, ATYPE_RW, acName2300_04, 0, &Obj.LASF_Config_1.lasf_1_offset_force},
};
const _objd SDO2301[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName2301_00, 4, NULL},
  {0x01, DTYPE_UNSIGNED8, 8, ATYPE_RW, acName2301_01, 1, &Obj.LASF_Config_2.lasf_2_operating_mode},
  {0x02, DTYPE_UNSIGNED16, 16, ATYPE_RW, acName2301_02, 2000, &Obj.LASF_Config_2.lasf_2_stroke_upper_limit},
  {0x03, DTYPE_UNSIGNED16, 16, ATYPE_RW, acName2301_03, 0, &Obj.LASF_Config_2.lasf_2_stroke_lower_limit},
  {0x04, DTYPE_INTEGER16, 16, ATYPE_RW, acName2301_04, 0, &Obj.LASF_Config_2.lasf_2_offset_force},
};
const _objd SDO2302[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName2302_00, 4, NULL},
  {0x01, DTYPE_UNSIGNED8, 8, ATYPE_RW, acName2302_01, 1, &Obj.LASF_Config_3.lasf_3_operating_mode},
  {0x02, DTYPE_UNSIGNED16, 16, ATYPE_RW, acName2302_02, 2000, &Obj.LASF_Config_3.lasf_3_stroke_upper_limit},
  {0x03, DTYPE_UNSIGNED16, 16, ATYPE_RW, acName2302_03, 0, &Obj.LASF_Config_3.lasf_3_stroke_lower_limit},
  {0x04, DTYPE_INTEGER16, 16, ATYPE_RW, acName2302_04, 0, &Obj.LASF_Config_3.lasf_3_offset_force},
};
const _objd SDO2303[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName2303_00, 4, NULL},
  {0x01, DTYPE_UNSIGNED8, 8, ATYPE_RW, acName2303_01, 1, &Obj.LASF_Config_4.lasf_4_operating_mode},
  {0x02, DTYPE_UNSIGNED16, 16, ATYPE_RW, acName2303_02, 2000, &Obj.LASF_Config_4.lasf_4_stroke_upper_limit},
  {0x03, DTYPE_UNSIGNED16, 16, ATYPE_RW, acName2303_03, 0, &Obj.LASF_Config_4.lasf_4_stroke_lower_limit},
  {0x04, DTYPE_INTEGER16, 16, ATYPE_RW, acName2303_04, 0, &Obj.LASF_Config_4.lasf_4_offset_force},
};
const _objd SDO2304[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName2304_00, 4, NULL},
  {0x01, DTYPE_UNSIGNED8, 8, ATYPE_RW, acName2304_01, 1, &Obj.LASF_Config_5.lasf_5_operating_mode},
  {0x02, DTYPE_UNSIGNED16, 16, ATYPE_RW, acName2304_02, 2000, &Obj.LASF_Config_5.lasf_5_stroke_upper_limit},
  {0x03, DTYPE_UNSIGNED16, 16, ATYPE_RW, acName2304_03, 0, &Obj.LASF_Config_5.lasf_5_stroke_lower_limit},
  {0x04, DTYPE_INTEGER16, 16, ATYPE_RW, acName2304_04, 0, &Obj.LASF_Config_5.lasf_5_offset_force},
};
const _objd SDO2305[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName2305_00, 4, NULL},
  {0x01, DTYPE_UNSIGNED8, 8, ATYPE_RW, acName2305_01, 1, &Obj.LASF_Config_6.lasf_6_operating_mode},
  {0x02, DTYPE_UNSIGNED16, 16, ATYPE_RW, acName2305_02, 2000, &Obj.LASF_Config_6.lasf_6_stroke_upper_limit},
  {0x03, DTYPE_UNSIGNED16, 16, ATYPE_RW, acName2305_03, 0, &Obj.LASF_Config_6.lasf_6_stroke_lower_limit},
  {0x04, DTYPE_INTEGER16, 16, ATYPE_RW, acName2305_04, 0, &Obj.LASF_Config_6.lasf_6_offset_force},
};
const _objd SDO2306[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName2306_00, 4, NULL},
  {0x01, DTYPE_UNSIGNED8, 8, ATYPE_RW, acName2306_01, 1, &Obj.LASF_Config_7.lasf_7_operating_mode},
  {0x02, DTYPE_UNSIGNED16, 16, ATYPE_RW, acName2306_02, 2000, &Obj.LASF_Config_7.lasf_7_stroke_upper_limit},
  {0x03, DTYPE_UNSIGNED16, 16, ATYPE_RW, acName2306_03, 0, &Obj.LASF_Config_7.lasf_7_stroke_lower_limit},
  {0x04, DTYPE_INTEGER16, 16, ATYPE_RW, acName2306_04, 0, &Obj.LASF_Config_7.lasf_7_offset_force},
};
const _objd SDO2307[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName2307_00, 4, NULL},
  {0x01, DTYPE_UNSIGNED8, 8, ATYPE_RW, acName2307_01, 1, &Obj.LASF_Config_8.lasf_8_operating_mode},
  {0x02, DTYPE_UNSIGNED16, 16, ATYPE_RW, acName2307_02, 2000, &Obj.LASF_Config_8.lasf_8_stroke_upper_limit},
  {0x03, DTYPE_UNSIGNED16, 16, ATYPE_RW, acName2307_03, 0, &Obj.LASF_Config_8.lasf_8_stroke_lower_limit},
  {0x04, DTYPE_INTEGER16, 16, ATYPE_RW, acName2307_04, 0, &Obj.LASF_Config_8.lasf_8_offset_force},
};
const _objd SDO2308[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName2308_00, 4, NULL},
  {0x01, DTYPE_UNSIGNED8, 8, ATYPE_RW, acName2308_01, 1, &Obj.LASF_Config_9.lasf_9_operating_mode},
  {0x02, DTYPE_UNSIGNED16, 16, ATYPE_RW, acName2308_02, 2000, &Obj.LASF_Config_9.lasf_9_stroke_upper_limit},
  {0x03, DTYPE_UNSIGNED16, 16, ATYPE_RW, acName2308_03, 0, &Obj.LASF_Config_9.lasf_9_stroke_lower_limit},
  {0x04, DTYPE_INTEGER16, 16, ATYPE_RW, acName2308_04, 0, &Obj.LASF_Config_9.lasf_9_offset_force},
};
const _objd SDO6000[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName6000_00, 9, NULL},
  {0x01, DTYPE_INTEGER8, 8, ATYPE_RO, acName6000_01, 0, &Obj.Tactile_T.tactile_1_fx},
  {0x02, DTYPE_INTEGER8, 8, ATYPE_RO, acName6000_02, 0, &Obj.Tactile_T.tactile_1_fy},
  {0x03, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName6000_03, 0, &Obj.Tactile_T.tactile_1_fz},
  {0x04, DTYPE_INTEGER8, 8, ATYPE_RO, acName6000_04, 0, &Obj.Tactile_T.tactile_2_fx},
  {0x05, DTYPE_INTEGER8, 8, ATYPE_RO, acName6000_05, 0, &Obj.Tactile_T.tactile_2_fy},
  {0x06, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName6000_06, 0, &Obj.Tactile_T.tactile_2_fz},
  {0x07, DTYPE_INTEGER8, 8, ATYPE_RO, acName6000_07, 0, &Obj.Tactile_T.tactile_3_fx},
  {0x08, DTYPE_INTEGER8, 8, ATYPE_RO, acName6000_08, 0, &Obj.Tactile_T.tactile_3_fy},
  {0x09, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName6000_09, 0, &Obj.Tactile_T.tactile_3_fz},
};
const _objd SDO6100[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName6100_00, 7, NULL},
  {0x01, DTYPE_INTEGER32, 32, ATYPE_RO, acName6100_01, 0, &Obj.Dynamixel_1_T.dxl_1_present_position},
  {0x02, DTYPE_INTEGER16, 16, ATYPE_RO, acName6100_02, 0, &Obj.Dynamixel_1_T.dxl_1_present_current},
  {0x03, DTYPE_INTEGER8, 8, ATYPE_RO, acName6100_03, 0, &Obj.Dynamixel_1_T.dxl_1_present_temperature},
  {0x04, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName6100_04, 0, &Obj.Dynamixel_1_T.dxl_1_hardware_error},
  {0x05, DTYPE_INTEGER32, 32, ATYPE_RO, acName6100_05, 0, &Obj.Dynamixel_1_T.dxl_1_present_velocity},
  {0x06, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName6100_06, 0, &Obj.Dynamixel_1_T.dxl_1_moving},
  {0x07, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName6100_07, 0, &Obj.Dynamixel_1_T.dxl_1_moving_status},
};
const _objd SDO6101[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName6101_00, 7, NULL},
  {0x01, DTYPE_INTEGER32, 32, ATYPE_RO, acName6101_01, 0, &Obj.Dynamixel_2_T.dxl_2_present_position},
  {0x02, DTYPE_INTEGER16, 16, ATYPE_RO, acName6101_02, 0, &Obj.Dynamixel_2_T.dxl_2_present_current},
  {0x03, DTYPE_INTEGER8, 8, ATYPE_RO, acName6101_03, 0, &Obj.Dynamixel_2_T.dxl_2_present_temperature},
  {0x04, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName6101_04, 0, &Obj.Dynamixel_2_T.dxl_2_hardware_error},
  {0x05, DTYPE_INTEGER32, 32, ATYPE_RO, acName6101_05, 0, &Obj.Dynamixel_2_T.dxl_2_present_velocity},
  {0x06, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName6101_06, 0, &Obj.Dynamixel_2_T.dxl_2_moving},
  {0x07, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName6101_07, 0, &Obj.Dynamixel_2_T.dxl_2_moving_status},
};
const _objd SDO6102[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName6102_00, 7, NULL},
  {0x01, DTYPE_INTEGER32, 32, ATYPE_RO, acName6102_01, 0, &Obj.Dynamixel_3_T.dxl_3_present_position},
  {0x02, DTYPE_INTEGER16, 16, ATYPE_RO, acName6102_02, 0, &Obj.Dynamixel_3_T.dxl_3_present_current},
  {0x03, DTYPE_INTEGER8, 8, ATYPE_RO, acName6102_03, 0, &Obj.Dynamixel_3_T.dxl_3_present_temperature},
  {0x04, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName6102_04, 0, &Obj.Dynamixel_3_T.dxl_3_hardware_error},
  {0x05, DTYPE_INTEGER32, 32, ATYPE_RO, acName6102_05, 0, &Obj.Dynamixel_3_T.dxl_3_present_velocity},
  {0x06, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName6102_06, 0, &Obj.Dynamixel_3_T.dxl_3_moving},
  {0x07, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName6102_07, 0, &Obj.Dynamixel_3_T.dxl_3_moving_status},
};
const _objd SDO6200[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName6200_00, 7, NULL},
  {0x01, DTYPE_INTEGER16, 16, ATYPE_RO, acName6200_01, 0, &Obj.LASF_1_T.lasf_1_actual_position},
  {0x02, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName6200_02, 0, &Obj.LASF_1_T.lasf_1_actual_current},
  {0x03, DTYPE_INTEGER16, 16, ATYPE_RO, acName6200_03, 0, &Obj.LASF_1_T.lasf_1_actual_force},
  {0x04, DTYPE_INTEGER8, 8, ATYPE_RO, acName6200_04, 0, &Obj.LASF_1_T.lasf_1_temperature},
  {0x05, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName6200_05, 0, &Obj.LASF_1_T.lasf_1_error_code},
  {0x06, DTYPE_INTEGER16, 16, ATYPE_RO, acName6200_06, 0, &Obj.LASF_1_T.lasf_1_target_position},
  {0x07, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName6200_07, 0, &Obj.LASF_1_T.lasf_1_force_adc},
};
const _objd SDO6201[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName6201_00, 7, NULL},
  {0x01, DTYPE_INTEGER16, 16, ATYPE_RO, acName6201_01, 0, &Obj.LASF_2_T.lasf_2_actual_position},
  {0x02, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName6201_02, 0, &Obj.LASF_2_T.lasf_2_actual_current},
  {0x03, DTYPE_INTEGER16, 16, ATYPE_RO, acName6201_03, 0, &Obj.LASF_2_T.lasf_2_actual_force},
  {0x04, DTYPE_INTEGER8, 8, ATYPE_RO, acName6201_04, 0, &Obj.LASF_2_T.lasf_2_temperature},
  {0x05, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName6201_05, 0, &Obj.LASF_2_T.lasf_2_error_code},
  {0x06, DTYPE_INTEGER16, 16, ATYPE_RO, acName6201_06, 0, &Obj.LASF_2_T.lasf_2_target_position},
  {0x07, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName6201_07, 0, &Obj.LASF_2_T.lasf_2_force_adc},
};
const _objd SDO6202[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName6202_00, 7, NULL},
  {0x01, DTYPE_INTEGER16, 16, ATYPE_RO, acName6202_01, 0, &Obj.LASF_3_T.lasf_3_actual_position},
  {0x02, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName6202_02, 0, &Obj.LASF_3_T.lasf_3_actual_current},
  {0x03, DTYPE_INTEGER16, 16, ATYPE_RO, acName6202_03, 0, &Obj.LASF_3_T.lasf_3_actual_force},
  {0x04, DTYPE_INTEGER8, 8, ATYPE_RO, acName6202_04, 0, &Obj.LASF_3_T.lasf_3_temperature},
  {0x05, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName6202_05, 0, &Obj.LASF_3_T.lasf_3_error_code},
  {0x06, DTYPE_INTEGER16, 16, ATYPE_RO, acName6202_06, 0, &Obj.LASF_3_T.lasf_3_target_position},
  {0x07, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName6202_07, 0, &Obj.LASF_3_T.lasf_3_force_adc},
};
const _objd SDO6203[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName6203_00, 7, NULL},
  {0x01, DTYPE_INTEGER16, 16, ATYPE_RO, acName6203_01, 0, &Obj.LASF_4_T.lasf_4_actual_position},
  {0x02, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName6203_02, 0, &Obj.LASF_4_T.lasf_4_actual_current},
  {0x03, DTYPE_INTEGER16, 16, ATYPE_RO, acName6203_03, 0, &Obj.LASF_4_T.lasf_4_actual_force},
  {0x04, DTYPE_INTEGER8, 8, ATYPE_RO, acName6203_04, 0, &Obj.LASF_4_T.lasf_4_temperature},
  {0x05, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName6203_05, 0, &Obj.LASF_4_T.lasf_4_error_code},
  {0x06, DTYPE_INTEGER16, 16, ATYPE_RO, acName6203_06, 0, &Obj.LASF_4_T.lasf_4_target_position},
  {0x07, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName6203_07, 0, &Obj.LASF_4_T.lasf_4_force_adc},
};
const _objd SDO6204[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName6204_00, 7, NULL},
  {0x01, DTYPE_INTEGER16, 16, ATYPE_RO, acName6204_01, 0, &Obj.LASF_5_T.lasf_5_actual_position},
  {0x02, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName6204_02, 0, &Obj.LASF_5_T.lasf_5_actual_current},
  {0x03, DTYPE_INTEGER16, 16, ATYPE_RO, acName6204_03, 0, &Obj.LASF_5_T.lasf_5_actual_force},
  {0x04, DTYPE_INTEGER8, 8, ATYPE_RO, acName6204_04, 0, &Obj.LASF_5_T.lasf_5_temperature},
  {0x05, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName6204_05, 0, &Obj.LASF_5_T.lasf_5_error_code},
  {0x06, DTYPE_INTEGER16, 16, ATYPE_RO, acName6204_06, 0, &Obj.LASF_5_T.lasf_5_target_position},
  {0x07, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName6204_07, 0, &Obj.LASF_5_T.lasf_5_force_adc},
};
const _objd SDO6205[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName6205_00, 7, NULL},
  {0x01, DTYPE_INTEGER16, 16, ATYPE_RO, acName6205_01, 0, &Obj.LASF_6_T.lasf_6_actual_position},
  {0x02, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName6205_02, 0, &Obj.LASF_6_T.lasf_6_actual_current},
  {0x03, DTYPE_INTEGER16, 16, ATYPE_RO, acName6205_03, 0, &Obj.LASF_6_T.lasf_6_actual_force},
  {0x04, DTYPE_INTEGER8, 8, ATYPE_RO, acName6205_04, 0, &Obj.LASF_6_T.lasf_6_temperature},
  {0x05, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName6205_05, 0, &Obj.LASF_6_T.lasf_6_error_code},
  {0x06, DTYPE_INTEGER16, 16, ATYPE_RO, acName6205_06, 0, &Obj.LASF_6_T.lasf_6_target_position},
  {0x07, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName6205_07, 0, &Obj.LASF_6_T.lasf_6_force_adc},
};
const _objd SDO6206[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName6206_00, 7, NULL},
  {0x01, DTYPE_INTEGER16, 16, ATYPE_RO, acName6206_01, 0, &Obj.LASF_7_T.lasf_7_actual_position},
  {0x02, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName6206_02, 0, &Obj.LASF_7_T.lasf_7_actual_current},
  {0x03, DTYPE_INTEGER16, 16, ATYPE_RO, acName6206_03, 0, &Obj.LASF_7_T.lasf_7_actual_force},
  {0x04, DTYPE_INTEGER8, 8, ATYPE_RO, acName6206_04, 0, &Obj.LASF_7_T.lasf_7_temperature},
  {0x05, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName6206_05, 0, &Obj.LASF_7_T.lasf_7_error_code},
  {0x06, DTYPE_INTEGER16, 16, ATYPE_RO, acName6206_06, 0, &Obj.LASF_7_T.lasf_7_target_position},
  {0x07, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName6206_07, 0, &Obj.LASF_7_T.lasf_7_force_adc},
};
const _objd SDO6207[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName6207_00, 7, NULL},
  {0x01, DTYPE_INTEGER16, 16, ATYPE_RO, acName6207_01, 0, &Obj.LASF_8_T.lasf_8_actual_position},
  {0x02, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName6207_02, 0, &Obj.LASF_8_T.lasf_8_actual_current},
  {0x03, DTYPE_INTEGER16, 16, ATYPE_RO, acName6207_03, 0, &Obj.LASF_8_T.lasf_8_actual_force},
  {0x04, DTYPE_INTEGER8, 8, ATYPE_RO, acName6207_04, 0, &Obj.LASF_8_T.lasf_8_temperature},
  {0x05, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName6207_05, 0, &Obj.LASF_8_T.lasf_8_error_code},
  {0x06, DTYPE_INTEGER16, 16, ATYPE_RO, acName6207_06, 0, &Obj.LASF_8_T.lasf_8_target_position},
  {0x07, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName6207_07, 0, &Obj.LASF_8_T.lasf_8_force_adc},
};
const _objd SDO6208[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName6208_00, 7, NULL},
  {0x01, DTYPE_INTEGER16, 16, ATYPE_RO, acName6208_01, 0, &Obj.LASF_9_T.lasf_9_actual_position},
  {0x02, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName6208_02, 0, &Obj.LASF_9_T.lasf_9_actual_current},
  {0x03, DTYPE_INTEGER16, 16, ATYPE_RO, acName6208_03, 0, &Obj.LASF_9_T.lasf_9_actual_force},
  {0x04, DTYPE_INTEGER8, 8, ATYPE_RO, acName6208_04, 0, &Obj.LASF_9_T.lasf_9_temperature},
  {0x05, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName6208_05, 0, &Obj.LASF_9_T.lasf_9_error_code},
  {0x06, DTYPE_INTEGER16, 16, ATYPE_RO, acName6208_06, 0, &Obj.LASF_9_T.lasf_9_target_position},
  {0x07, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName6208_07, 0, &Obj.LASF_9_T.lasf_9_force_adc},
};
const _objd SDO6300[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName6300_00, 6, NULL},
  {0x01, DTYPE_INTEGER16, 16, ATYPE_RO, acName6300_01, 0, &Obj.AFT150_T.ft_fx},
  {0x02, DTYPE_INTEGER16, 16, ATYPE_RO, acName6300_02, 0, &Obj.AFT150_T.ft_fy},
  {0x03, DTYPE_INTEGER16, 16, ATYPE_RO, acName6300_03, 0, &Obj.AFT150_T.ft_fz},
  {0x04, DTYPE_INTEGER16, 16, ATYPE_RO, acName6300_04, 0, &Obj.AFT150_T.ft_mx},
  {0x05, DTYPE_INTEGER16, 16, ATYPE_RO, acName6300_05, 0, &Obj.AFT150_T.ft_my},
  {0x06, DTYPE_INTEGER16, 16, ATYPE_RO, acName6300_06, 0, &Obj.AFT150_T.ft_mz},
};
const _objd SDO7000[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName7000_00, 4, NULL},
  {0x01, DTYPE_INTEGER32, 32, ATYPE_RW, acName7000_01, 0, &Obj.Dynamixel_1_R.dxl_1_goal_position},
  {0x02, DTYPE_INTEGER16, 16, ATYPE_RW, acName7000_02, 0, &Obj.Dynamixel_1_R.dxl_1_goal_current},
  {0x03, DTYPE_INTEGER32, 32, ATYPE_RW, acName7000_03, 0, &Obj.Dynamixel_1_R.dxl_1_goal_velocity},
  {0x04, DTYPE_UNSIGNED8, 8, ATYPE_RW, acName7000_04, 0, &Obj.Dynamixel_1_R.dxl_1_torque_enable},
};
const _objd SDO7001[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName7001_00, 4, NULL},
  {0x01, DTYPE_INTEGER32, 32, ATYPE_RW, acName7001_01, 0, &Obj.Dynamixel_2_R.dxl_2_goal_position},
  {0x02, DTYPE_INTEGER16, 16, ATYPE_RW, acName7001_02, 0, &Obj.Dynamixel_2_R.dxl_2_goal_current},
  {0x03, DTYPE_INTEGER32, 32, ATYPE_RW, acName7001_03, 0, &Obj.Dynamixel_2_R.dxl_2_goal_velocity},
  {0x04, DTYPE_UNSIGNED8, 8, ATYPE_RW, acName7001_04, 0, &Obj.Dynamixel_2_R.dxl_2_torque_enable},
};
const _objd SDO7002[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName7002_00, 4, NULL},
  {0x01, DTYPE_INTEGER32, 32, ATYPE_RW, acName7002_01, 0, &Obj.Dynamixel_3_R.dxl_3_goal_position},
  {0x02, DTYPE_INTEGER16, 16, ATYPE_RW, acName7002_02, 0, &Obj.Dynamixel_3_R.dxl_3_goal_current},
  {0x03, DTYPE_INTEGER32, 32, ATYPE_RW, acName7002_03, 0, &Obj.Dynamixel_3_R.dxl_3_goal_velocity},
  {0x04, DTYPE_UNSIGNED8, 8, ATYPE_RW, acName7002_04, 0, &Obj.Dynamixel_3_R.dxl_3_torque_enable},
};
const _objd SDO7100[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName7100_00, 3, NULL},
  {0x01, DTYPE_INTEGER16, 16, ATYPE_RW, acName7100_01, 0, &Obj.LASF_1_R.lasf_1_target_position},
  {0x02, DTYPE_INTEGER16, 16, ATYPE_RW, acName7100_02, 0, &Obj.LASF_1_R.lasf_1_target_force},
  {0x03, DTYPE_UNSIGNED16, 16, ATYPE_RW, acName7100_03, 0, &Obj.LASF_1_R.lasf_1_target_speed},
};
const _objd SDO7101[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName7101_00, 3, NULL},
  {0x01, DTYPE_INTEGER16, 16, ATYPE_RW, acName7101_01, 0, &Obj.LASF_2_R.lasf_2_target_position},
  {0x02, DTYPE_INTEGER16, 16, ATYPE_RW, acName7101_02, 0, &Obj.LASF_2_R.lasf_2_target_force},
  {0x03, DTYPE_UNSIGNED16, 16, ATYPE_RW, acName7101_03, 0, &Obj.LASF_2_R.lasf_2_target_speed},
};
const _objd SDO7102[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName7102_00, 3, NULL},
  {0x01, DTYPE_INTEGER16, 16, ATYPE_RW, acName7102_01, 0, &Obj.LASF_3_R.lasf_3_target_position},
  {0x02, DTYPE_INTEGER16, 16, ATYPE_RW, acName7102_02, 0, &Obj.LASF_3_R.lasf_3_target_force},
  {0x03, DTYPE_UNSIGNED16, 16, ATYPE_RW, acName7102_03, 0, &Obj.LASF_3_R.lasf_3_target_speed},
};
const _objd SDO7103[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName7103_00, 3, NULL},
  {0x01, DTYPE_INTEGER16, 16, ATYPE_RW, acName7103_01, 0, &Obj.LASF_4_R.lasf_4_target_position},
  {0x02, DTYPE_INTEGER16, 16, ATYPE_RW, acName7103_02, 0, &Obj.LASF_4_R.lasf_4_target_force},
  {0x03, DTYPE_UNSIGNED16, 16, ATYPE_RW, acName7103_03, 0, &Obj.LASF_4_R.lasf_4_target_speed},
};
const _objd SDO7104[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName7104_00, 3, NULL},
  {0x01, DTYPE_INTEGER16, 16, ATYPE_RW, acName7104_01, 0, &Obj.LASF_5_R.lasf_5_target_position},
  {0x02, DTYPE_INTEGER16, 16, ATYPE_RW, acName7104_02, 0, &Obj.LASF_5_R.lasf_5_target_force},
  {0x03, DTYPE_UNSIGNED16, 16, ATYPE_RW, acName7104_03, 0, &Obj.LASF_5_R.lasf_5_target_speed},
};
const _objd SDO7105[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName7105_00, 3, NULL},
  {0x01, DTYPE_INTEGER16, 16, ATYPE_RW, acName7105_01, 0, &Obj.LASF_6_R.lasf_6_target_position},
  {0x02, DTYPE_INTEGER16, 16, ATYPE_RW, acName7105_02, 0, &Obj.LASF_6_R.lasf_6_target_force},
  {0x03, DTYPE_UNSIGNED16, 16, ATYPE_RW, acName7105_03, 0, &Obj.LASF_6_R.lasf_6_target_speed},
};
const _objd SDO7106[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName7106_00, 3, NULL},
  {0x01, DTYPE_INTEGER16, 16, ATYPE_RW, acName7106_01, 0, &Obj.LASF_7_R.lasf_7_target_position},
  {0x02, DTYPE_INTEGER16, 16, ATYPE_RW, acName7106_02, 0, &Obj.LASF_7_R.lasf_7_target_force},
  {0x03, DTYPE_UNSIGNED16, 16, ATYPE_RW, acName7106_03, 0, &Obj.LASF_7_R.lasf_7_target_speed},
};
const _objd SDO7107[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName7107_00, 3, NULL},
  {0x01, DTYPE_INTEGER16, 16, ATYPE_RW, acName7107_01, 0, &Obj.LASF_8_R.lasf_8_target_position},
  {0x02, DTYPE_INTEGER16, 16, ATYPE_RW, acName7107_02, 0, &Obj.LASF_8_R.lasf_8_target_force},
  {0x03, DTYPE_UNSIGNED16, 16, ATYPE_RW, acName7107_03, 0, &Obj.LASF_8_R.lasf_8_target_speed},
};
const _objd SDO7108[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName7108_00, 3, NULL},
  {0x01, DTYPE_INTEGER16, 16, ATYPE_RW, acName7108_01, 0, &Obj.LASF_9_R.lasf_9_target_position},
  {0x02, DTYPE_INTEGER16, 16, ATYPE_RW, acName7108_02, 0, &Obj.LASF_9_R.lasf_9_target_force},
  {0x03, DTYPE_UNSIGNED16, 16, ATYPE_RW, acName7108_03, 0, &Obj.LASF_9_R.lasf_9_target_speed},
};
const _objd SDO7200[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName7200_00, 9, NULL},
  {0x01, DTYPE_UNSIGNED8, 8, ATYPE_RW, acName7200_01, 0, &Obj.INDICATOR_R.led_1_red},
  {0x02, DTYPE_UNSIGNED8, 8, ATYPE_RW, acName7200_02, 0, &Obj.INDICATOR_R.led_1_green},
  {0x03, DTYPE_UNSIGNED8, 8, ATYPE_RW, acName7200_03, 0, &Obj.INDICATOR_R.led_1_blue},
  {0x04, DTYPE_UNSIGNED8, 8, ATYPE_RW, acName7200_04, 0, &Obj.INDICATOR_R.led_2_red},
  {0x05, DTYPE_UNSIGNED8, 8, ATYPE_RW, acName7200_05, 0, &Obj.INDICATOR_R.led_2_green},
  {0x06, DTYPE_UNSIGNED8, 8, ATYPE_RW, acName7200_06, 0, &Obj.INDICATOR_R.led_2_blue},
  {0x07, DTYPE_UNSIGNED8, 8, ATYPE_RW, acName7200_07, 0, &Obj.INDICATOR_R.led_3_red},
  {0x08, DTYPE_UNSIGNED8, 8, ATYPE_RW, acName7200_08, 0, &Obj.INDICATOR_R.led_3_green},
  {0x09, DTYPE_UNSIGNED8, 8, ATYPE_RW, acName7200_09, 0, &Obj.INDICATOR_R.led_3_blue},
};

const _objectlist SDOobjects[] =
{
  {0x1000, OTYPE_VAR, 0, 0, acName1000, SDO1000},
  {0x1008, OTYPE_VAR, 0, 0, acName1008, SDO1008},
  {0x1009, OTYPE_VAR, 0, 0, acName1009, SDO1009},
  {0x100A, OTYPE_VAR, 0, 0, acName100A, SDO100A},
  {0x1018, OTYPE_RECORD, 4, 0, acName1018, SDO1018},
  {0x1600, OTYPE_RECORD, 4, 0, acName1600, SDO1600},
  {0x1601, OTYPE_RECORD, 4, 0, acName1601, SDO1601},
  {0x1602, OTYPE_RECORD, 4, 0, acName1602, SDO1602},
  {0x1603, OTYPE_RECORD, 3, 0, acName1603, SDO1603},
  {0x1604, OTYPE_RECORD, 3, 0, acName1604, SDO1604},
  {0x1605, OTYPE_RECORD, 3, 0, acName1605, SDO1605},
  {0x1606, OTYPE_RECORD, 3, 0, acName1606, SDO1606},
  {0x1607, OTYPE_RECORD, 3, 0, acName1607, SDO1607},
  {0x1608, OTYPE_RECORD, 3, 0, acName1608, SDO1608},
  {0x1609, OTYPE_RECORD, 3, 0, acName1609, SDO1609},
  {0x160A, OTYPE_RECORD, 3, 0, acName160A, SDO160A},
  {0x160B, OTYPE_RECORD, 3, 0, acName160B, SDO160B},
  {0x160C, OTYPE_RECORD, 9, 0, acName160C, SDO160C},
  {0x1A00, OTYPE_RECORD, 9, 0, acName1A00, SDO1A00},
  {0x1A01, OTYPE_RECORD, 7, 0, acName1A01, SDO1A01},
  {0x1A02, OTYPE_RECORD, 7, 0, acName1A02, SDO1A02},
  {0x1A03, OTYPE_RECORD, 7, 0, acName1A03, SDO1A03},
  {0x1A04, OTYPE_RECORD, 7, 0, acName1A04, SDO1A04},
  {0x1A05, OTYPE_RECORD, 7, 0, acName1A05, SDO1A05},
  {0x1A06, OTYPE_RECORD, 7, 0, acName1A06, SDO1A06},
  {0x1A07, OTYPE_RECORD, 7, 0, acName1A07, SDO1A07},
  {0x1A08, OTYPE_RECORD, 7, 0, acName1A08, SDO1A08},
  {0x1A09, OTYPE_RECORD, 7, 0, acName1A09, SDO1A09},
  {0x1A0A, OTYPE_RECORD, 7, 0, acName1A0A, SDO1A0A},
  {0x1A0B, OTYPE_RECORD, 7, 0, acName1A0B, SDO1A0B},
  {0x1A0C, OTYPE_RECORD, 7, 0, acName1A0C, SDO1A0C},
  {0x1A0D, OTYPE_RECORD, 6, 0, acName1A0D, SDO1A0D},
  {0x1C00, OTYPE_ARRAY, 4, 0, acName1C00, SDO1C00},
  {0x1C12, OTYPE_ARRAY, 13, 0, acName1C12, SDO1C12},
  {0x1C13, OTYPE_ARRAY, 14, 0, acName1C13, SDO1C13},
  {0x2000, OTYPE_RECORD, 3, 0, acName2000, SDO2000},
  {0x2100, OTYPE_RECORD, 9, 0, acName2100, SDO2100},
  {0x2200, OTYPE_RECORD, 6, 0, acName2200, SDO2200},
  {0x2300, OTYPE_RECORD, 4, 0, acName2300, SDO2300},
  {0x2301, OTYPE_RECORD, 4, 0, acName2301, SDO2301},
  {0x2302, OTYPE_RECORD, 4, 0, acName2302, SDO2302},
  {0x2303, OTYPE_RECORD, 4, 0, acName2303, SDO2303},
  {0x2304, OTYPE_RECORD, 4, 0, acName2304, SDO2304},
  {0x2305, OTYPE_RECORD, 4, 0, acName2305, SDO2305},
  {0x2306, OTYPE_RECORD, 4, 0, acName2306, SDO2306},
  {0x2307, OTYPE_RECORD, 4, 0, acName2307, SDO2307},
  {0x2308, OTYPE_RECORD, 4, 0, acName2308, SDO2308},
  {0x6000, OTYPE_RECORD, 9, 0, acName6000, SDO6000},
  {0x6100, OTYPE_RECORD, 7, 0, acName6100, SDO6100},
  {0x6101, OTYPE_RECORD, 7, 0, acName6101, SDO6101},
  {0x6102, OTYPE_RECORD, 7, 0, acName6102, SDO6102},
  {0x6200, OTYPE_RECORD, 7, 0, acName6200, SDO6200},
  {0x6201, OTYPE_RECORD, 7, 0, acName6201, SDO6201},
  {0x6202, OTYPE_RECORD, 7, 0, acName6202, SDO6202},
  {0x6203, OTYPE_RECORD, 7, 0, acName6203, SDO6203},
  {0x6204, OTYPE_RECORD, 7, 0, acName6204, SDO6204},
  {0x6205, OTYPE_RECORD, 7, 0, acName6205, SDO6205},
  {0x6206, OTYPE_RECORD, 7, 0, acName6206, SDO6206},
  {0x6207, OTYPE_RECORD, 7, 0, acName6207, SDO6207},
  {0x6208, OTYPE_RECORD, 7, 0, acName6208, SDO6208},
  {0x6300, OTYPE_RECORD, 6, 0, acName6300, SDO6300},
  {0x7000, OTYPE_RECORD, 4, 0, acName7000, SDO7000},
  {0x7001, OTYPE_RECORD, 4, 0, acName7001, SDO7001},
  {0x7002, OTYPE_RECORD, 4, 0, acName7002, SDO7002},
  {0x7100, OTYPE_RECORD, 3, 0, acName7100, SDO7100},
  {0x7101, OTYPE_RECORD, 3, 0, acName7101, SDO7101},
  {0x7102, OTYPE_RECORD, 3, 0, acName7102, SDO7102},
  {0x7103, OTYPE_RECORD, 3, 0, acName7103, SDO7103},
  {0x7104, OTYPE_RECORD, 3, 0, acName7104, SDO7104},
  {0x7105, OTYPE_RECORD, 3, 0, acName7105, SDO7105},
  {0x7106, OTYPE_RECORD, 3, 0, acName7106, SDO7106},
  {0x7107, OTYPE_RECORD, 3, 0, acName7107, SDO7107},
  {0x7108, OTYPE_RECORD, 3, 0, acName7108, SDO7108},
  {0x7200, OTYPE_RECORD, 9, 0, acName7200, SDO7200},
  {0xffff, 0xff, 0xff, 0xff, NULL, NULL}
};
