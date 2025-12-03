/**
    ****************************(C) COPYRIGHT 2025 群星战队****************************
    * @file       chassis_power_control_with_supercap.c/h
    * @brief      这里是桂林理工大学群星战队使用超级电容之后底盘功率控制的库，
    *             代码使用西交利物浦的电机功率模型与功率控制开源改编而成
    * @note       
    * @history
    *   版本          日期           作者                     变更
    *   V1.0.2516     2025-4-20     魔法电容使者              1. 完成
    *
    @verbatim
    ==============================================================================

    ==============================================================================
    @endverbatim
    ****************************(C) COPYRIGHT 2025 群星战队****************************
*/

#ifndef CHASSIS_POWER_CONTROL_WITH_SUPERCAP_H
#define CHASSIS_POWER_CONTROL_WITH_SUPERCAP_H


#ifdef __cplusplus
extern "C" {
#endif

#include "robot_def.h"

/**
 * @brief          限制功率，主要限制电机电流
 * @param[in]      chassis_power_control: 底盘数据
 * @retval         none
 */
CAP_Ctrl_Chassis_s *chassis_power_control_with_supercap(Chassis_Upload_CAP_s *chassis_fetch_data);
void set_chassis_power(float power);
#ifdef __cplusplus
}
#endif

#endif

