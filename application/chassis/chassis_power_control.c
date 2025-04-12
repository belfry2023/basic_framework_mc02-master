#include "chassis_power_control.h"
#include "message_center.h"
#include "arm_math.h"
#include "robot_def.h"
#include "remote_control.h"
#include "controller.h"


// extern cap_measure_t cap_measure; // capacitor data structure
// extern RC_ctrl_t rc_ctrl;
// uint8_t cap_state = 0;

static Publisher_t *chassis_power_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
static Chassis_Power_Data_s chassis_power_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等
static uint8_t cap_state = 0; // capacitor state

static uint16_t max_power_limit = 40;
static float32_t chassis_max_power = 0;
static float input_power = 0;		 // input power from battery (referee system)
static float initial_give_power[4]; // initial power from PID calculation
static float initial_total_power = 0;

static float32_t scaled_give_power[4];
static float32_t chassis_power = 0.0f;
static float32_t chassis_power_buffer = 0.0f;

static 	float32_t toque_coefficient = 1.99688994e-6f; // (20/16384)*(0.3)*(187/3591)/9.55
static 	float32_t a = 1.23e-07;						 // k1
static 	float32_t k2 = 1.453e-07;					 // k2
static 	float32_t constant = 4.081f;

static PIDInstance power_buffer_pid;

/// @brief 
/// @param  
void chassis_power_control_init(void)
{
    PID_Init_Config_s pid_cfg = {
        .Kp = 1,
        .Ki = 0,
        .Kd = 0,
        .DeadBand = 1,
        .MaxOut = 30,
        .MaxOut_ = 30,
        .IntegralLimit = 5,
        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_DerivativeFilter,
    };
    PIDInit(&power_buffer_pid, &pid_cfg);
    chassis_power_pub = PubRegister("power_cmd", sizeof(Chassis_Power_Data_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
}


/// @brief 
/// @param  
void chassis_power_control(void)
{

    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);

    chassis_power_buffer += (chassis_fetch_data.chassis_power - chassis_max_power) / 200;
    PIDCalculate(&power_buffer_pid, chassis_power, 30);
	chassis_max_power = chassis_fetch_data.chassis_power_limit; // get the max power from referee system
	input_power = chassis_max_power - power_buffer_pid.Output; // Input power floating at maximum power

	chassis_power_send.chassis_power_limit = input_power; // set the max power to chassis

	if(chassis_fetch_data.chassis_cap_current > 5)
	{
		if (cap_state == 0)
			chassis_max_power = input_power + 5; // Slightly greater than the maximum power, avoiding the capacitor being full all the time and improving energy utilization
		else
			chassis_max_power = input_power + 200;
	}

	for(uint8_t i = 0; i < 4; i++)
	{
		initial_give_power[i] = chassis_fetch_data.motor_current[i] * toque_coefficient * chassis_fetch_data.motor_speed[i] 
		+ k2 * chassis_fetch_data.motor_speed[i] * chassis_fetch_data.motor_speed[i] 
		+ a * chassis_fetch_data.motor_current[i] * chassis_fetch_data.motor_current[i]
		+ constant;
		if(initial_give_power < 0)
		{
			continue;
			initial_total_power += initial_give_power[i];
		}
		if(initial_total_power > chassis_max_power)
		{
			float32_t power_scale = chassis_max_power;
			for(uint8_t i = 0; i < 4; i++)
			{
				scaled_give_power[i] = initial_give_power[i] * power_scale;
				if(scaled_give_power[i] < 0)
					continue;
				float32_t b = toque_coefficient * chassis_fetch_data.motor_speed[i];
				float32_t c = a * chassis_fetch_data.motor_speed[i] * chassis_fetch_data.motor_speed[i] - scaled_give_power[i] + constant;
				float32_t inside = b * b - 4 * a * c;
				if (inside < 0)
					continue;
				else if (chassis_fetch_data.motor_current[i] > 0) // Selection of the calculation formula according to the direction of the original moment
				{
					float32_t temp = (-b + sqrt(inside)) / (2 * a);
					if (temp > 15000)
						chassis_power_send.motor_current[i] = 15000;
					else
						chassis_power_send.motor_current[i] = temp;
				}
				else
				{
					float32_t temp = (-b - sqrt(inside)) / (2 * a);
					if (temp < -15000)
						chassis_power_send.motor_current[i] = -15000;
					else
						chassis_power_send.motor_current[i] = temp;
				}
			}
		}
	}
	PubPushMessage(chassis_power_pub, (void *)&chassis_power_send);
}