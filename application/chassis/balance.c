#include "balance.h"
#include "message_center.h"
#include "LK9025.h"
#include "go_motor.h"
#include "robot_def.h"
#include "ins_task.h"
#include "usart.h"
static Publisher_t *chassis_pub;                    // 用于发布底盘的数据
static Subscriber_t *chassis_sub;                   // 用于订阅底盘的控制命令

static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的控制命令
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据

static LKMotorInstance *motor_l, *motor_r;
static GOMotorInstance *leg_lf, *leg_lb, *leg_rf, *leg_rb;
static attitude_t *chassis_IMU_data; 

void balance_init()
{
    chassis_IMU_data = INS_Init();
    Motor_Init_Config_s wheel_motor_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 1,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 10,
                .Ki = 2,
                .Kd = 0,
                .DeadBand = 0.1,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 100,
                .MaxOut =1000,
            },
            .speed_PID = {
                .Kp = 10,
                .Ki = 0,
                .Kd = 2,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 3000,
                .MaxOut = 20000,
            },
            .other_angle_feedback_ptr = &chassis_IMU_data->Roll,
            .other_speed_feedback_ptr = &chassis_IMU_data->Gyro[0],
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
        },
        .motor_type = LK9025,
    };
    motor_l = LKMotorInit(&wheel_motor_config);
    wheel_motor_config.can_init_config.tx_id = 2;
    wheel_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_r = LKMotorInit(&wheel_motor_config);

    Other_Motor_Init_Config_s leg_motor = {
        .usart_init_config = {
            .usart_handle = &huart2
        },
        .conf = {
            .ID = 1,
            .Kp = 2,
            .Kd = 1,
            .Mode = foc,
        },
        .motor_type = GOM8010,
    };
    leg_lf = GOmotorInit(leg_motor);
    leg_motor.conf.ID = 2;
    leg_lb = GOmotorInit(leg_motor);
    leg_motor.conf.ID = 3;
    leg_rf = GOmotorInit(leg_motor);
    leg_motor.conf.ID = 4;
    leg_rb = GOmotorInit(leg_motor);
    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
}


void balance_task()
{
    // SubGetMessage(chassis_sub, &chassis_cmd_recv);
    // LKMotorSetRef(motor_l, 0);
    // LKMotorSetRef(motor_r, 0);
    // PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
}
