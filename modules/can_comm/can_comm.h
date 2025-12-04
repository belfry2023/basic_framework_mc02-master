#ifndef __CAN_COMM_H
#define __CAN_COMM_H

#include "bsp_can.h"
#include "daemon.h"

// 数据类型枚举
typedef enum {
    CAN_DATA_UINT8_ARRAY,  // 8个uint8_t
    CAN_DATA_FLOAT_2,      // 2个float
    CAN_DATA_MIXED         // 混合类型（用户定义的结构体）
} CANDataType;

// 数据联合体，用于uint8_t数组和float之间的转换
typedef union {
    uint8_t bytes[8];      // 8个字节
    struct {
        float f1;          // 第一个float
        float f2;          // 第二个float
    } floats;
    struct {
        uint32_t u32_1;    // 第一个32位数据
        uint32_t u32_2;    // 第二个32位数据
    } u32s;
    int16_t i16s[4];       // 4个int16_t
    uint16_t u16s[4];      // 4个uint16_t
} CANDataUnion;

// 数据更新回调函数类型
typedef void (*CANCommDataUpdateCallback)(void *instance);

// CAN通信初始化配置结构体
typedef struct {
    CAN_Init_Config_s can_config;  // CAN配置
    uint32_t rx_id;                // 接收ID
    uint32_t tx_id;                // 发送ID
    uint8_t target_struct_len;     // 目标结构体长度（字节）
    CANCommDataUpdateCallback data_update_callback;  // 数据更新回调
} CANComm_Init_Config_s;

// CAN通信实例结构体
typedef struct {
    CANInstance *can_ins;          // CAN实例
    
    // ID配置
    uint32_t rx_id;                // 接收ID
    uint32_t tx_id;                // 发送ID
    
    // 数据转换
    CANDataUnion data_union;       // 数据联合体，用于转换
    uint8_t target_struct[8];           // 目标结构体指针
    uint8_t target_struct_len;     // 目标结构体长度
    
    // 状态标志
    uint8_t update_flag;           // 数据更新标志
    uint32_t last_update_time;     // 最后更新时间（毫秒）
    
    // 回调函数
    CANCommDataUpdateCallback data_update_callback;  // 数据更新回调
} CANCommInstance;

// 函数声明
CANCommInstance *CANCommInit(CANComm_Init_Config_s *comm_config);

// 发送函数
void CANCommSend(CANCommInstance *instance, void *data, CANDataType data_type);
void CANCommSendFloat2(CANCommInstance *instance, float f1, float f2);
void CANCommSendUint8Array(CANCommInstance *instance, uint8_t *data);

// 接收函数
uint8_t CANCommGet(CANCommInstance *instance, CANDataType data_type, void *output);
uint8_t CANCommGetFloat2(CANCommInstance *instance, float *f1, float *f2);
uint8_t CANCommGetUint8Array(CANCommInstance *instance, uint8_t *output);

// 辅助函数
void *CANCommGetTargetStruct(CANCommInstance *instance);
CANDataUnion *CANCommGetDataUnion(CANCommInstance *instance);
uint8_t CANCommIsUpdated(CANCommInstance *instance);
void CANCommSetUpdateCallback(CANCommInstance *instance, CANCommDataUpdateCallback callback);
uint32_t CANCommGetLastUpdateTime(CANCommInstance *instance);
void CANCommDeinit(CANCommInstance *instance);

#endif /* __CAN_COMM_H */