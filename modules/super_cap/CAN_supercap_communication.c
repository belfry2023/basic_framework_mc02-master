/**
  ****************************(C) COPYRIGHT 2025 群星战队****************************
  * @file       CAN_supercap_communication.c/h
  * @brief      这里是群星战队超级电容数据定义、发送、接收与解析的库
  * @note       
  * @history
  *   版本          日期           作者                     变更
  *   V1.0.2516     2025-4-20     魔法电容使              1. 完成
	* 	V1.1.2517	    2025-4-25     魔法电容使				      2. 添加功率计算，新增功率补偿
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2025 群星战队****************************
*/

#include "CAN_supercap_communication.h"
#include "main.h"
#include "stdint.h"
#include <string.h>
#include <stdlib.h>
static SuperCap_s *SuperCap = NULL;

/**
  * @brief          获取超级电容在线状态，1在线，0离线
  * @retval         none
  */
uint8_t get_supercap_online_state(void){

  SuperCap->NowCapTick = HAL_GetTick(); //更新时间戳
  SuperCap->DeltaCapTick = SuperCap->NowCapTick - SuperCap->LastCapTick; //计算时间差

	if(SuperCap->CAN_SuperCapRXData.SuperCapReady == READY){
		//比赛开始的时候，开始统计消耗的能量
		SuperCap->chassis_energy_in_gamming += SuperCap->CAN_SuperCapRXData.BatPower * SuperCap->DeltaCapTick *0.001f;
    // 因为STM32的系统定时器是1ms的周期，所以*0.001，单位化为秒（S），能量单位才是焦耳（J）
	}

  if(SuperCap->DeltaCapTick > 1000){
    //如果时间差大于1s，说明超电信号丢失，返回超电离线的标志
    return 0;
  }else {
    //如果时间差小于1s，说明超电信号正常，返回超电在线的标志
    return 1;
  }
}

/**
  * @brief          获取根据超级电容功率统计的底盘消耗能量，单位：焦耳（J）
  * @retval         none
  */
uint32_t get_chassis_energy_from_supercap(void){

  return SuperCap->chassis_energy_in_gamming;
}


//----------超级电容数据从CAN传到结构体------------------
void transfer_SuperCap_measure(uint8_t *data)
{
  SuperCap->LastCapTick = HAL_GetTick();
  SuperCap->CAN_SuperCapRXData.SuperCapReady = (SuperCapReadyEnum)data[0];
  SuperCap->CAN_SuperCapRXData.SuperCapState = (SuperCapStateEnum)data[1];
  SuperCap->CAN_SuperCapRXData.SuperCapEnergy = data[2];
  SuperCap->CAN_SuperCapRXData.ChassisPower = data[3] << 1; //左移一位是为了扩大范围，超电发出来的的时候右移了一位
  SuperCap->CAN_SuperCapRXData.BatVoltage = data[4];
  SuperCap->CAN_SuperCapRXData.BatPower = data[5];
}

static void SuperCapRxCallback(CANInstance *_instance)
{
    uint8_t *rxbuff;
    rxbuff = (uint8_t*)_instance->rx_buff;
    // get_supercap_online_state();
    transfer_SuperCap_measure(rxbuff);
}


/**
  * @brief          获取超级电容的运行状态，具体查看枚举SuperCapStateEnum
  * @retval         none
  */
SuperCapStateEnum get_supercap_state(void){
  return (SuperCapStateEnum)SuperCap->CAN_SuperCapRXData.SuperCapState;
}

/**
  * @brief          获取超级电容读到的电池电压，单位伏（V）
  * @retval         none
  */
float get_battery_voltage_from_supercap(void){
  return (float)SuperCap->CAN_SuperCapRXData.BatVoltage * 0.1f;
}

/**
  * @brief          获取超级电容可用能量，范围：0-100%
  * @retval         none
  */
uint8_t get_supercap_energy(void){
  return SuperCap->CAN_SuperCapRXData.SuperCapEnergy;
}

/**
  * @brief          设置超级电容功率补偿
  * @retval         none
  */
void set_supercap_power_offset(uint8_t offset){
  SuperCap->PowerOffset = offset;
}

/**
  * @brief          发送超级电容数据
  * @param[in]      Enable: 超级电容使能
  * @param[in]      Charge: 超级电容充电，在PLUS版本中无效
  * @param[in]      PowerLimit: 裁判系统功率限制
  * @param[in]      Chargepower: 超级电容充电功率，在PLUS版本中无效
  * @retval         none
  */
void SuperCapSend(SuperCap_s *instance)
{
  instance->can_ins->tx_buff[0] = SuperCap->TX_Temp.Enable;
  instance->can_ins->tx_buff[1] = SuperCap->TX_Temp.Charge;
  instance->can_ins->tx_buff[2] = SuperCap->TX_Temp.Powerlimit - SuperCap->PowerOffset;
  instance->can_ins->tx_buff[3] = SuperCap->TX_Temp.ChargePower;
  CANTransmit(instance->can_ins,1);
}

SuperCap_s *SuperCapInit(FDCAN_HandleTypeDef *can_handle)
{
  SuperCap = (SuperCap_s *)malloc(sizeof(SuperCap_s));
  memset(SuperCap, 0, sizeof(SuperCap_s));
  CAN_Init_Config_s can_config = {
    .can_handle = can_handle,
    .id = SUPERCAP_TX_ID,
    .rx_id = SUPERCAP_RX_ID,
    .tx_id = SUPERCAP_TX_ID,
    .can_module_callback = SuperCapRxCallback
  };
  
  SuperCap->can_ins = CANRegister(&can_config);
  return SuperCap;
}