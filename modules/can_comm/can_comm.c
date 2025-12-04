#include "can_comm.h"
#include "memory.h"
#include "stdlib.h"
#include "bsp_dwt.h"
#include "bsp_log.h"

/**
 * @brief cancomm的接收回调函数
 * @note 只要ID匹配就直接解包，无需帧头帧尾校验
 */
static void CANCommRxCallback(CANInstance *_instance)
{
    CANCommInstance *comm = (CANCommInstance *)_instance->id;
    
    // 检查是否是我们要接收的ID（由硬件过滤器确保）
    if (_instance->rx_len == 8)  // CAN标准帧数据长度为8字节
    {
        // 直接将数据拷贝到数据联合体
        memcpy(comm->data_union.bytes, _instance->rx_buff, 8);
        
        // 如果用户定义了目标结构体，则拷贝到目标结构体
        if (comm->target_struct_len > 0 && comm->target_struct_len <= 8)
        {
            memcpy(comm->target_struct, comm->data_union.bytes, comm->target_struct_len);
        }
        
        comm->update_flag = 1;  // 设置更新标志
        comm->last_update_time = DWT_GetTimeline_ms();  // 记录更新时间
        
        // 如果有数据更新回调，则调用
        if (comm->data_update_callback != NULL)
        {
            comm->data_update_callback(comm);
        }
    }
}

/**
 * @brief 初始化CAN通信实例
 * @param comm_config 初始化配置
 * @return CANCommInstance指针
 */
CANCommInstance *CANCommInit(CANComm_Init_Config_s *comm_config)
{
    CANCommInstance *ins = (CANCommInstance *)malloc(sizeof(CANCommInstance));
    memset(ins, 0, sizeof(CANCommInstance));
    
    // 保存接收和发送ID
    ins->rx_id = comm_config->rx_id;
    ins->tx_id = comm_config->tx_id;
    ins->target_struct_len = comm_config->target_struct_len;
    
    
    // 保存数据更新回调
    if (comm_config->data_update_callback != NULL)
    {
        ins->data_update_callback = comm_config->data_update_callback;
    }
    
    // 设置CAN实例
    comm_config->can_config.id = ins;
    comm_config->can_config.can_module_callback = CANCommRxCallback;
    // comm_config->can_config.rx_id = comm_config->rx_id;  // 设置接收ID
    
    ins->can_ins = CANRegister(&comm_config->can_config);
    
    return ins;
}

/**
 * @brief 发送数据（两个float，8字节）
 * @param instance CAN通信实例
 * @param data 数据指针，可以是两个float或8个uint8_t
 * @param data_type 数据类型
 */
void CANCommSend(CANCommInstance *instance, void *data, CANDataType data_type)
{
    if (instance->can_ins->id == 0)
    {
        LOGERROR("[can_comm] TX ID not set!");
        return;
    }
    
    CANDataUnion send_union;
    
    // 根据数据类型填充发送联合体
    switch (data_type)
    {
        case CAN_DATA_UINT8_ARRAY:
            // data应该是uint8_t[8]
            memcpy(send_union.bytes, data, 8);
            break;
            
        case CAN_DATA_FLOAT_2:
            // data应该是float[2]
            send_union.floats.f1 = ((float*)data)[0];
            send_union.floats.f2 = ((float*)data)[1];
            break;
            
        case CAN_DATA_MIXED:
            // data应该是用户自定义的混合结构体，需要保证总大小不超过8字节
            memcpy(send_union.bytes, data, 8);
            break;
            
        default:
            LOGERROR("[can_comm] Unknown data type: %d", data_type);
            return;
    }
    
    // 设置发送ID
    // instance->can_ins->tx_id = instance->tx_id;
    
    // 将数据拷贝到CAN发送缓冲区
    memcpy(instance->can_ins->tx_buff, send_union.bytes, 8);
    
    // 发送数据（固定8字节）
    CANTransmit(instance->can_ins, 1);
    
    DWT_Delay(0.0001); // 适当延时
}

/**
 * @brief 发送两个float数据
 * @param instance CAN通信实例
 * @param f1 第一个float
 * @param f2 第二个float
 */
void CANCommSendFloat2(CANCommInstance *instance, float f1, float f2)
{
    if (instance->tx_id == 0)
    {
        LOGERROR("[can_comm] TX ID not set!");
        return;
    }
    
    CANDataUnion send_union;
    send_union.floats.f1 = f1;
    send_union.floats.f2 = f2;
    
    // 设置发送ID
    instance->can_ins->tx_id = instance->tx_id;
    
    // 将数据拷贝到CAN发送缓冲区
    memcpy(instance->can_ins->tx_buff, send_union.bytes, 8);
    
    // 发送数据（固定8字节）
    CANTransmit(instance->can_ins, 1);
    
    DWT_Delay(0.0001); // 适当延时
}

/**
 * @brief 发送uint8_t数组
 * @param instance CAN通信实例
 * @param data uint8_t数组（8个元素）
 */
void CANCommSendUint8Array(CANCommInstance *instance, uint8_t *data)
{
    if (instance->tx_id == 0)
    {
        LOGERROR("[can_comm] TX ID not set!");
        return;
    }
    
    // 设置发送ID
    instance->can_ins->tx_id = instance->tx_id;
    
    // 将数据拷贝到CAN发送缓冲区
    memcpy(instance->can_ins->tx_buff, data, 8);
    
    // 发送数据（固定8字节）
    CANTransmit(instance->can_ins, 1);
    
    DWT_Delay(0.0001); // 适当延时
}

/**
 * @brief 获取接收到的数据
 * @param instance CAN通信实例
 * @param data_type 期望的数据类型
 * @param output 输出缓冲区指针
 * @return 成功返回1，失败返回0
 */
uint8_t CANCommGet(CANCommInstance *instance, CANDataType data_type, void *output)
{
    if (!instance->update_flag)
        return 0;
    
    switch (data_type)
    {
        case CAN_DATA_UINT8_ARRAY:
            // 输出8个uint8_t
            memcpy(output, instance->data_union.bytes, 8);
            break;
            
        case CAN_DATA_FLOAT_2:
            // 输出2个float
            ((float*)output)[0] = instance->data_union.floats.f1;
            ((float*)output)[1] = instance->data_union.floats.f2;
            break;
            
        case CAN_DATA_MIXED:
            // 输出目标结构体（如果已定义）
            if (instance->target_struct_len > 0)
            {
                memcpy(output, instance->target_struct, instance->target_struct_len);
            }
            else
            {
                // 如果没有定义目标结构体，返回原始字节
                memcpy(output, instance->data_union.bytes, 8);
            }
            break;
            
        default:
            return 0;
    }
    
    instance->update_flag = 0; // 读取后清除标志
    return 1;
}

/**
 * @brief 获取两个float数据
 * @param instance CAN通信实例
 * @param f1 输出第一个float的指针
 * @param f2 输出第二个float的指针
 * @return 成功返回1，失败返回0
 */
uint8_t CANCommGetFloat2(CANCommInstance *instance, float *f1, float *f2)
{
    if (!instance->update_flag)
        return 0;
    
    *f1 = instance->data_union.floats.f1;
    *f2 = instance->data_union.floats.f2;
    
    instance->update_flag = 0;
    return 1;
}

/**
 * @brief 获取uint8_t数组
 * @param instance CAN通信实例
 * @param output 输出缓冲区（至少8字节）
 * @return 成功返回1，失败返回0
 */
uint8_t CANCommGetUint8Array(CANCommInstance *instance, uint8_t *output)
{
    if (!instance->update_flag)
        return 0;
    
    memcpy(output, instance->data_union.bytes, 8);
    
    instance->update_flag = 0;
    return 1;
}

/**
 * @brief 获取目标结构体指针
 * @param instance CAN通信实例
 * @return 目标结构体指针
 */
void *CANCommGetTargetStruct(CANCommInstance *instance)
{
    return instance->target_struct;
}

/**
 * @brief 获取数据联合体指针（用于直接访问转换后的数据）
 * @param instance CAN通信实例
 * @return 数据联合体指针
 */
CANDataUnion *CANCommGetDataUnion(CANCommInstance *instance)
{
    return &instance->data_union;
}

/**
 * @brief 检查是否有新数据
 * @param instance CAN通信实例
 * @return 有新数据返回1，否则返回0
 */
uint8_t CANCommIsUpdated(CANCommInstance *instance)
{
    return instance->update_flag;
}

/**
 * @brief 设置数据更新回调函数
 * @param instance CAN通信实例
 * @param callback 回调函数
 */
void CANCommSetUpdateCallback(CANCommInstance *instance, CANCommDataUpdateCallback callback)
{
    instance->data_update_callback = callback;
}

/**
 * @brief 获取最后更新时间
 * @param instance CAN通信实例
 * @return 最后更新时间（毫秒）
 */
uint32_t CANCommGetLastUpdateTime(CANCommInstance *instance)
{
    return instance->last_update_time;
}

/**
 * @brief 释放CAN通信实例
 * @param instance CAN通信实例指针
 */
void CANCommDeinit(CANCommInstance *instance)
{
    if (instance->target_struct != NULL)
    {
        free(instance->target_struct);
    }
    free(instance);
}