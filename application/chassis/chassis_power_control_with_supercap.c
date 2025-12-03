/**
    ****************************(C) COPYRIGHT 2025 群星战队****************************
    * @file       chassis_power_control_with_supercap.c/h
    * @brief      这里是群星战队使用超级电容之后底盘功率控制的库，
    *             代码使用西交利物浦的电机功率模型与功率控制开源改编而成
    * @note       
    * @history
    *   版本          日期           作者                     变更
    *   V1.0.2516     2025-4-20     魔法电容使              1. 完成
	* 	V1.1.2517	  2025-4-25     魔法电容使				2. 添加功率计算，新增功率补偿
    *
    @verbatim
    ==============================================================================

    ==============================================================================
    @endverbatim
    ****************************(C) COPYRIGHT 2025 群星战队****************************
*/

#include "chassis_power_control_with_supercap.h"
#include "CAN_supercap_communication.h"
#include "arm_math.h"

#define WARRING_ENERGY_BUFFER 50 

static float ChassisSetPower = 0;

static CAP_Ctrl_Chassis_s chassis_cap_send;

void set_chassis_power(float power)
{
    ChassisSetPower = power;
}

CAP_Ctrl_Chassis_s *chassis_power_control_with_supercap(Chassis_Upload_CAP_s *chassis_fetch_data)
{

    uint8_t RefereePowerLimit =0;
	float ChassisMaxPower = 0;
    
	float InitialGivePower[4]; // initial power from PID calculation
	float InitialTotalPower = 0;
	float ScaledGivePower[4];

	float chassis_energy_buffer = 0.0f;

	float toque_coefficient = 1.99688994e-6f;   // (20/16384)*(0.3)*(187/3591)/9.55
	float k1 = 1.23e-07;					 	// k1
	float k2 = 1.453e-07;					 	// k2
	float constant = 4.081f;

	float power_scale =0;
	float eneygy_scale =0;
	uint8_t PowerOffset =0;

    // get_chassis_power_limit(&RefereePowerLimit);
    // get_chassis_power_and_buffer(NULL, &chassis_energy_buffer);
	RefereePowerLimit = chassis_fetch_data->chassis_power_mx;
	chassis_energy_buffer = chassis_fetch_data->buffer_energy;
	chassis_energy_buffer = 100;

    if(get_supercap_online_state()){


		if(chassis_fetch_data->SuperCapReady == READY){
			if(chassis_fetch_data->ChassisPower > ChassisSetPower){
				// 这里是超电正常运行的时候，超电可以随时补偿底盘功率，故底盘的功率可以超出功率限制。
				// 如果超电检测的底盘功率超出了设定值，则说明功率模型有误差，需要对模型进行一定的修正
				// 这是为了确保底盘功率能够按照设定的功率运行。
				power_scale = ChassisSetPower / (float)chassis_fetch_data->ChassisPower;
				ChassisMaxPower = ChassisSetPower * power_scale;
			}else {
				ChassisMaxPower = ChassisSetPower;
			}
		}else {
			// 这里是超电处于保护状态的时候，超电无法补偿功率，故底盘功率只能使用裁判系统的功率。
			// 这里使用超电反馈的功率进行功率限制，减小对裁判系统依赖，保证没有裁判系统的时候仍然可以 以指定的功率去运行。
			if(chassis_fetch_data->ChassisPower > RefereePowerLimit){
				// 这里是使用超电的时候，如果超电检测的底盘功率超出了设定值，则说明功率模型有误差，需要对模型进行一定的修正
				power_scale = (float)RefereePowerLimit / (float)chassis_fetch_data->ChassisPower;
				ChassisMaxPower = (float)RefereePowerLimit * power_scale;
			}else {
				ChassisMaxPower = RefereePowerLimit;
			}
		}
        
        if(chassis_energy_buffer < WARRING_ENERGY_BUFFER){
			//超电在线的时候，经过测试，拟合校准过的数据，经过50CM的18AWG线带载5A，与裁判系统的功率误差小于5W
			//另外，PLUS版本拥有远端补偿的设计，经过补偿，与裁判系统的功率误差< 1W
			//所以，如果缓冲能量被意外消耗，说明相对误差仍然存在，需要小幅度修正，如果补偿5W之后仍超功率，则说明拟合参数有问题。
			PowerOffset = (WARRING_ENERGY_BUFFER - chassis_energy_buffer) * 0.2f; //这里根据缓冲能量剩余的量去做补偿，最大补偿10W
        }else {
			PowerOffset =0;
        }

		set_supercap_power_offset(PowerOffset);
		ChassisMaxPower = ChassisMaxPower - PowerOffset;//缓冲能量减少的时候，底盘功率也减掉一点（因为超电功率误差也可能导致超功率）

    }else {
        // 这里是超电离线的时候，需要使用缓冲能量对电机功率模型进行修正，避免功率模型误差导致超功率

        if(chassis_energy_buffer < WARRING_ENERGY_BUFFER){
            eneygy_scale = chassis_energy_buffer * 0.02f; //0.02是1/50，转成乘法，优化一点点速度。
            ChassisMaxPower = (float)RefereePowerLimit * eneygy_scale;
        }else {
            ChassisMaxPower = RefereePowerLimit;
        }
    }

	//以下是西交利物浦的电机模型功率控制。

	for (uint8_t i = 0; i < 4; i++) // first get all the initial motor power and total motor power
	{
		InitialGivePower[i] = chassis_fetch_data->motor[i].speed_pid_out * toque_coefficient * chassis_fetch_data->motor[i].spd +
								k2 * chassis_fetch_data->motor[i].spd * chassis_fetch_data->motor[i].spd +
								k1 * chassis_fetch_data->motor[i].speed_pid_out * chassis_fetch_data->motor[i].speed_pid_out + constant;

		if (InitialGivePower < 0) // negative power not included (transitory)
			continue;
		InitialTotalPower += InitialGivePower[i];
	}

	if (InitialTotalPower > ChassisMaxPower) // determine if larger than max power
	{
		float power_scale = ChassisMaxPower / InitialTotalPower;
		for (uint8_t i = 0; i < 4; i++)
		{
			ScaledGivePower[i] = InitialGivePower[i] * power_scale; // get scaled power
			if (ScaledGivePower[i] < 0)
			{
				continue;
			}

			float b = toque_coefficient * chassis_fetch_data->motor[i].spd;
			float c = k2 * chassis_fetch_data->motor[i].spd * chassis_fetch_data->motor[i].spd - ScaledGivePower[i] + constant;
			float inside = b * b - 4 * k1 * c;

			if (inside < 0)
			{
				continue;
			}
			else if (chassis_fetch_data->motor[i].speed_pid_out > 0) // Selection of the calculation formula according to the direction of the original moment
			{
				float temp = (-b + sqrt(inside)) / (2 * k1);
				if (temp > 16000)
				{
					chassis_cap_send.motor[i].speed_pid_out = 16000;
				}
				else
					chassis_cap_send.motor[i].speed_pid_out = temp;
			}
			else
			{
				float temp = (-b - sqrt(inside)) / (2 * k1);
				if (temp < -16000)
				{
					chassis_cap_send.motor[i].speed_pid_out = -16000;
				}
				else
					chassis_cap_send.motor[i].speed_pid_out = temp;
			}
		}
	}
	return &chassis_cap_send;
}
