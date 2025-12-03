/**
    ****************************(C) COPYRIGHT 2025 群星战队****************************
    * @file       CAN_supercap_communication.c/h
    * @brief      这里是群星战队超级电容数据定义、发送、接收与解析的库
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

#ifndef CAN_SUPERCAP_COMMUNICATION_H
#define CAN_SUPERCAP_COMMUNICATION_H

#ifdef __cplusplus
extern "C" {
#endif


#include "bsp_can.h"
#include "stm32h723xx.h"


#define SUPERCAP_TX_ID 0x001  //C板发给超级电容的ID
#define SUPERCAP_RX_ID 0x100  //超级电容发给C板的ID


//超级电容的状态标志位，超级电容运行或者保护的具体状态反馈
typedef enum
{
	DISCHARGE =0 ,	//放电状态
	CHARGE =1,	//充电状态
	WAIT =2,	//待机状态
	SOFTSTART_PROTECTION =3,//处于软起动状态
	OVER_LOAD_PROTECTION = 4,	//超电过载保护状态
	BAT_OVER_VOLTAGE_PROTECTION =5,	//过压保护状态
	BAT_UNDER_VOLTAGE_PROTECTION =6,	//电池欠压保护，电池要没电了，换电池
	CAP_UNDER_VOLTAGE_PROTECTION =7,	//超级电容欠压保护，超级电容用完电了，要充一会才能用
	OVER_TEMPERATURE_PROTECTION =8,	//过温保护，太热了
    BOOM = 9,  //超电爆炸了
}SuperCapStateEnum;

//超级电容准备状态，用于判断超级电容是否可以使用
typedef enum
{
	UNREADY =0 ,
	READY =1,
}SuperCapReadyEnum;

// 发送给超级电容的数据
typedef struct {
    FunctionalState Enable ; //超级电容使能。1使能，0失能
    SuperCapStateEnum Charge ; //V1.1超级电容充电。1充电，0放电，在PLUS版本中，此标志位无效，超电的充放电是自动的
    uint8_t Powerlimit;  //裁判系统功率限制
    uint8_t ChargePower;  //V1.1超级电容充电功率，在PLUS版本中，此参数，超电的充电功率随着底盘功率变化
}CAN_SuperCapTXDataTypeDef;

// 从超级电容接收到的数据
typedef struct {
    uint8_t SuperCapEnergy;//超级电容可用能量：0-100%
    uint16_t ChassisPower; //底盘功率，0-512，由于传输的时候为了扩大量程右移了一位，所以接收的时候需要左移还原（丢精度）。
    SuperCapReadyEnum SuperCapReady;//超级电容【可用标志】：1为可用，0为不可用
    SuperCapStateEnum SuperCapState;//超级电容【状态标志】：各个状态对应的状态码查看E_SuperCapState枚举。
    uint8_t BatVoltage; //通过超级电容监控电池电压*10，
    uint8_t BatPower;
}CAN_SuperCapRXDataTypeDef;

typedef struct
{
    CANInstance *can_ins;
    CAN_SuperCapTXDataTypeDef TX_Temp;
    CAN_SuperCapRXDataTypeDef CAN_SuperCapRXData;
    uint32_t LastCapTick; //上一次收到超电信号的时间戳
    uint32_t NowCapTick;  //现在收到超电信号的时间戳
    uint32_t DeltaCapTick; //两次收到超电信号的时间差
    uint32_t chassis_energy_in_gamming;
    uint8_t PowerOffset;
}SuperCap_s;


extern CAN_SuperCapRXDataTypeDef CAN_SuperCapRXData;

void set_supercap_power_offset(uint8_t offset);

// 以下函数是电控读取超电数据所需要调用的函数
void transfer_SuperCap_measure( uint8_t *data);
void CAN_cmd_SuperCap(CAN_SuperCapTXDataTypeDef * TX_Temp);
uint8_t get_supercap_online_state(void);
SuperCapStateEnum get_supercap_state(void);
float get_battery_voltage_from_supercap(void);
uint8_t get_supercap_energy(void);
uint32_t get_chassis_energy_from_supercap(void);
void SuperCapSend(SuperCap_s *instance);
SuperCap_s *SuperCapInit(FDCAN_HandleTypeDef *can_handle);
#ifdef __cplusplus
}
#endif

#endif // CAN_SUPERCAP_COMMUNICATION_H
