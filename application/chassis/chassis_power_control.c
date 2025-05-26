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

static uint16_t max_power_limit = 10;
static float32_t chassis_max_power = 0;
static float input_power = 0;		 // input power from battery (referee system)
static float allocate_give_power[4]; // initial power from PID calculation
static float allocate_total_power = 0;

static float32_t distribut_give_power[4];
static float32_t chassis_power = 0.0f;
static float32_t chassis_power_buffer = 0.0f;

static 	float32_t toque_coefficient = 0.00000199688994; // (20/16384)*(0.3)*(187/3591)/9.55
static 	float32_t k1 = 0.000000123;						 // k1
static 	float32_t k2 = 0.0000001453;					 // k2
static 	float32_t constant = 4.80f;

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
	for(size_t i = 0;i < 4; i++)
	{
        allocate_give_power[i] = toque_coefficient * chassis_fetch_data.motor_current[i] * chassis_fetch_data.motor_speed[i] + chassis_fetch_data.motor_current[i] * chassis_fetch_data.motor_current[i] * k1 + chassis_fetch_data.motor_speed[i] * k2 * chassis_fetch_data.motor_speed[i] + constant;
        allocate_total_power += allocate_give_power[i];
	}
    float32_t power_time = chassis_fetch_data.chassis_power_limit / allocate_total_power;
	for(size_t i = 0; i < 4; i++)
	{
        distribut_give_power[i] = allocate_give_power[i] * power_time;
		float32_t a = k1;
		float32_t b = chassis_fetch_data.motor_speed[i] * toque_coefficient;
		float32_t c = k2 * chassis_fetch_data.motor_speed[i] * chassis_fetch_data.motor_speed[i] + constant - 20;
        if(b * b - 4 * a * c > 0)
        {
            if((-b + sqrt(b * b - 4 * a * c)) / (2 * a) > 0)
            {
                chassis_power_send.motor_current_up[i] = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
                chassis_power_send.motor_current_down[i] = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
            }
            else
            {
                chassis_power_send.motor_current_down[i] = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
                chassis_power_send.motor_current_up[i] = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
            }
        }
	}
	PubPushMessage(chassis_power_pub, (void *)&chassis_power_send);
}