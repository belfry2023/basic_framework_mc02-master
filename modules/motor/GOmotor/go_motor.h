#ifndef GO_MOTOR_H
#define GO_MOTOR_H

#include "bsp_usart.h"
#include "motor_def.h"
#include <stdint.h>
#include "daemon.h"

#define GO_MOTOR_CNT 4

typedef enum 
{
    lock,
    foc,
    cali
};

typedef struct SetMode
{
    int16_t tqr_ft;
    int16_t spd_des;
    int pos_des;
    int16_t Kp;
    int16_t Kd;

    uint8_t ID;
    uint8_t Mode;
    
    /* data */
}Motor_Conf;


typedef struct messsure
{
    int16_t tqr_fb;
    int16_t rot_spd;
    int pos;
    char tem;
    char wrong;
    /* data */
}Motor_messure;
#pragma pack(1)
typedef struct motor_control
{
    uint8_t head[2];
    int8_t Set;
    int16_t tqr_set;
    int16_t spd_set;
    int pos_set;
    int16_t Kpos;
    int16_t Kspd;
    uint16_t crc;
    /* data */
}motor_ctrl;
#pragma pack(0)

typedef struct GOMotorInstance
{
    
    USARTInstance *motor_usart;

    Motor_messure mes;
    Motor_Conf ctrl_set;

    motor_ctrl ctrl_send;

    Motor_Type_e motor_type;        // 电机类型
    Motor_Working_Type_e stop_flag; // 启停标志

    uint8_t rs485_flag;

    DaemonInstance* daemon;
    unsigned int feed_cnt;
    float dt;
    /* data */
}GOMotorInstance;

typedef struct
{
    Motor_Conf conf;
    Motor_Type_e motor_type;
    USART_Init_Config_s usart_init_config;
} Other_Motor_Init_Config_s;


void GOMotorControlInit();
GOMotorInstance *GOmotorInit(Other_Motor_Init_Config_s config);


#endif // !GO_MOTOR_H


