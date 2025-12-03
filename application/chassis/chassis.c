#include "chassis.h"
#include "robot_def.h"
#include "dji_motor.h"
#include "super_cap.h"
#include "message_center.h"
#include "referee_task.h"
#include "ins_task.h"
#include "general_def.h"
#include "bsp_dwt.h"
#include "referee_UI.h"
#include "arm_math.h"
#include "buzzer.h"

/* 根据robot_def.h中的macro自动计算的参数 */
#define HALF_WHEEL_BASE (WHEEL_BASE / 2.0f)     // 半轴距
#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f)   // 半轮距
#define PERIMETER_WHEEL (RADIUS_WHEEL * 2 * PI) // 轮子周长

/* 底盘应用包含的模块和信息存储,底盘是单例模式,因此不需要为底盘建立单独的结构体 */
#ifdef CHASSIS_BOARD // 如果是底盘板,使用板载IMU获取底盘转动角速度
#include "can_comm.h"
#include "ins_task.h"
static CANCommInstance *chasiss_can_comm1; // 用于接收vx, vy
static CANCommInstance *chasiss_can_comm2; // 用于接收wz, offset_angle
static CANCommInstance *chasiss_can_comm3; // 用于接收chassis_mode
static Chassis_Ctrl_Cmd_1s chassis_cmd_1recv;         // 底盘接收到的x, y速度控制命令
static Chassis_Ctrl_Cmd_2s chassis_cmd_2recv;         // 底盘接收到的z, offset_angle控制命令
static Chassis_Ctrl_Cmd_3s chassis_cmd_3recv;         // 底盘接收到的控制模式命令
#endif // CHASSIS_BOARD

#ifdef ONE_BOARD
static Publisher_t *chassis_pub;                    // 用于发布底盘的数据
static Subscriber_t *chassis_sub;                   // 用于订阅底盘的控制命令
#endif // ONE_BOARD

// 完整的控制命令（用于单板模式）
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;         // 底盘接收到的完整控制命令

static Chassis_Power_Data_s chassis_power_recv;
static Chassis_Upload_Data_s chassis_feedback_data; // 底盘回传的反馈数据

static referee_info_t *referee_data;       // 用于获取裁判系统的数据
static Referee_Interactive_info_t ui_data; // UI数据，将底盘中的数据传入此结构体的对应变量中，UI会自动检测是否变化，对应显示UI

static SuperCapInstance *cap;                                       // 超级电容
static DJIMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb; // left right forward back

static PIDInstance chassis_follow_to_yaw_pid;
static BuzzzerInstance *buzzerc;

void ChassisInit()
{
    // 四个轮子的参数一样,改tx_id和反转标志位即可
    Motor_Init_Config_s chassis_motor_config = {
        .can_init_config.can_handle = &hcan1,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 10, // 4.5
                .Ki = 0,  // 0
                .Kd = 0,  // 0
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 12000,
                .MaxOut_ = -12000},
            .current_PID = {
                .Kp = 0.5, // 0.4
                .Ki = 0,   // 0
                .Kd = 0,
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 3000,
                .MaxOut_ = -3000
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
        },
        .motor_type = M3508,
    };
    
    PID_Init_Config_s chassis_follow_to_yaw_config = {
        .Kp = 50,
        .Ki = 0,
        .Kd = 5,
        .Derivative_LPF_RC = 0,
        .IntegralLimit = 3000,
        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_DerivativeFilter,
        .MaxOut = 15000,
        .MaxOut_ = -15000,
        .DeadBand = 3};

    PIDInit(&chassis_follow_to_yaw_pid, &chassis_follow_to_yaw_config);

    chassis_motor_config.can_init_config.tx_id = 2;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_lf = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 1;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_rf = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 4;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_lb = DJIMotorInit(&chassis_motor_config);

    chassis_motor_config.can_init_config.tx_id = 3;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_rb = DJIMotorInit(&chassis_motor_config);

    referee_data = UITaskInit(&huart1, &ui_data); // 裁判系统初始化,会同时初始化UI

    // 初始化蜂鸣器
    Buzzer_config_s buzzer = {
        .alarm_level = ALARM_LEVEL_HIGH,
        .loudness = 0,
        .octave = OCTAVE_2,
    };
    buzzerc = BuzzerRegister(&buzzer);
    
#ifdef ONE_BOARD // 单板控制整车,则通过pubsub来传递消息
    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_power_sub = SubRegister("power_cmd", sizeof(Chassis_Power_Data_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
#endif // ONE_BOARD
    
#ifdef CHASSIS_BOARD // 双板模式，使用CAN通信接收三个独立的结构体
    // 初始化第一个CAN通信实例，用于接收vx, vy
    CANComm_Init_Config_s comm_conf1 = {
        .can_config = {
            .can_handle = &hcan2,
            .tx_id = 0x311,  // 底盘发送ID
            .rx_id = 0x312,  // 接收云台发送的vx, vy数据
        },
        .target_struct_len = sizeof(Chassis_Ctrl_Cmd_1s),
    };
    chasiss_can_comm1 = CANCommInit(&comm_conf1);
    
    // 初始化第二个CAN通信实例，用于接收wz, offset_angle
    CANComm_Init_Config_s comm_conf2 = {
        .can_config = {
            .can_handle = &hcan2,
            .tx_id = 0x311,  // 底盘发送ID
            .rx_id = 0x313,  // 接收云台发送的wz, offset_angle数据
        },
        .target_struct_len = sizeof(Chassis_Ctrl_Cmd_2s),
    };
    chasiss_can_comm2 = CANCommInit(&comm_conf2);
    
    // 初始化第三个CAN通信实例，用于接收chassis_mode
    CANComm_Init_Config_s comm_conf3 = {
        .can_config = {
            .can_handle = &hcan2,
            .tx_id = 0x311,  // 底盘发送ID
            .rx_id = 0x314,  // 接收云台发送的chassis_mode数据
        },
        .target_struct_len = sizeof(Chassis_Ctrl_Cmd_3s),
    };
    chasiss_can_comm3 = CANCommInit(&comm_conf3);
#endif // CHASSIS_BOARD
}

#define LF_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RF_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define LB_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RB_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)

/**
 * @brief 计算每个轮毂电机的输出,正运动学解算
 */
static void MecanumCalculate()
{
    float vt_lf, vt_rf, vt_lb, vt_rb; // 底盘速度解算后的临时输出
    
    vt_lf = -chassis_cmd_recv.vx - chassis_cmd_recv.vy - chassis_cmd_recv.wz * LF_CENTER;
    vt_rf = -chassis_cmd_recv.vx + chassis_cmd_recv.vy - chassis_cmd_recv.wz * RF_CENTER;
    vt_lb = chassis_cmd_recv.vx - chassis_cmd_recv.vy - chassis_cmd_recv.wz * LB_CENTER;
    vt_rb = chassis_cmd_recv.vx + chassis_cmd_recv.vy - chassis_cmd_recv.wz * RB_CENTER;
    
    DJIMotorSetRef(motor_lf, vt_lf);
    DJIMotorSetRef(motor_rf, vt_rf);
    DJIMotorSetRef(motor_lb, vt_lb);
    DJIMotorSetRef(motor_rb, vt_rb);
}

/**
 * @brief 根据控制模式设定旋转速度
 */
static void UpdateChassisWZ()
{
    switch (chassis_cmd_recv.chassis_mode)
    {
    case CHASSIS_NO_FOLLOW: // 底盘不旋转,但维持全向机动
        chassis_cmd_recv.wz = 0;
        break;
    case CHASSIS_FOLLOW_GIMBAL_YAW: // 跟随云台
        // 这里可以根据offset_angle计算wz，需要云台角度反馈
        // 当前简单实现为固定值，实际应使用PID控制
        chassis_cmd_recv.wz = 1000; // 示例值
        break;
    case CHASSIS_ROTATE: // 自旋
        chassis_cmd_recv.wz = 4000; // 示例值
        break;
    default:
        chassis_cmd_recv.wz = 0;
        break;
    }
}

/**
 * @brief 合并三个独立的控制命令为一个完整的控制命令
 */
static void MergeChassisCommands()
{
#ifdef CHASSIS_BOARD
    // 从CAN通信实例获取三个独立的结构体
    if (CANCommGet(chasiss_can_comm1, CAN_DATA_MIXED, &chassis_cmd_1recv))
    {
        chassis_cmd_recv.vx = chassis_cmd_1recv.vx;
        chassis_cmd_recv.vy = chassis_cmd_1recv.vy;
    }
    
    if (CANCommGet(chasiss_can_comm2, CAN_DATA_MIXED, &chassis_cmd_2recv))
    {
        chassis_cmd_recv.wz = chassis_cmd_2recv.wz;
        chassis_cmd_recv.offset_angle = chassis_cmd_2recv.offset_angle;
    }
    
    if (CANCommGet(chasiss_can_comm3, CAN_DATA_MIXED, &chassis_cmd_3recv))
    {
        chassis_cmd_recv.chassis_mode = chassis_cmd_3recv.chassis_mode;
    }
#endif // CHASSIS_BOARD
}

/* 机器人底盘控制核心任务 */
void ChassisTask()
{
    // 获取控制信息
#ifdef ONE_BOARD
    // 单板模式：从发布-订阅系统获取完整控制命令
    SubGetMessage(chassis_sub, &chassis_cmd_recv);
    SubGetMessage(chassis_power_sub, &chassis_power_recv);
#endif // ONE_BOARD

#ifdef CHASSIS_BOARD
    // 双板模式：从CAN通信获取三个独立结构体并合并
    MergeChassisCommands();
#endif // CHASSIS_BOARD

    // 急停处理
    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE)
    {
        DJIMotorStop(motor_lf);
        DJIMotorStop(motor_rf);
        DJIMotorStop(motor_lb);
        DJIMotorStop(motor_rb);
        buzzerc->alarm_state = ALARM_ON;
    }
    else
    {
        DJIMotorEnable(motor_lf);
        DJIMotorEnable(motor_rf);
        DJIMotorEnable(motor_lb);
        DJIMotorEnable(motor_rb);
        buzzerc->alarm_state = ALARM_OFF;
    }

    // 根据控制模式更新wz
    UpdateChassisWZ();

    // 根据云台和底盘的角度offset将控制量映射到底盘坐标系上
    static float sin_theta, cos_theta;
    float chassis_vx, chassis_vy;
    
    cos_theta = arm_cos_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    sin_theta = arm_sin_f32(chassis_cmd_recv.offset_angle * DEGREE_2_RAD);
    chassis_vx = chassis_cmd_recv.vx * cos_theta - chassis_cmd_recv.vy * sin_theta;
    chassis_vy = chassis_cmd_recv.vx * sin_theta + chassis_cmd_recv.vy * cos_theta;
    
    // 更新控制命令中的vx, vy为底盘坐标系下的值
    chassis_cmd_recv.vx = chassis_vx;
    chassis_cmd_recv.vy = chassis_vy;

    // 进行正运动学解算，计算底盘输出
    MecanumCalculate();

    // 更新UI数据
    ui_data.chassis_mode = chassis_cmd_recv.chassis_mode;

    // 推送反馈消息
#ifdef ONE_BOARD
    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
#endif // ONE_BOARD

#ifdef CHASSIS_BOARD
    // 如果需要向云台板发送反馈数据，可以在这里添加
    // 注意：需要为每个反馈数据类型创建独立的CAN通信实例
#endif // CHASSIS_BOARD
}