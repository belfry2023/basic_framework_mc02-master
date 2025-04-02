#include "go_motor.h"
#include "memory.h"
#include "general_def.h"
#include "user_lib.h"
#include "cmsis_os.h"
#include "string.h"
#include "daemon.h"
#include "stdlib.h"
#include "bsp_log.h"
#include "crc16.h"
#include "crc_ccitt.h"
static uint8_t idx = 0; // register idx,是该文件的全局电机索引,在注册时使用
/* GO电机的实例,此处仅保存指针,内存的分配将通过电机实例初始化时通过malloc()进行 */
static GOMotorInstance *go_motor_instance[GO_MOTOR_CNT] = {NULL};// 会在control任务中遍历该指针数组进行pid计算
static osThreadId go_task_handle[GO_MOTOR_CNT];

static void DecodeGOIMotor(USARTInstance *_instance)
{
    
    uint16_t crc;
    if(_instance->recv_buff[0] == 0xfd && _instance->recv_buff[1] == 0xee)
    {
        for (size_t i = 0; i < GO_MOTOR_CNT; i++)
        {
            if(go_motor_instance[i]->ctrl_set.ID == _instance->recv_buff[2]>>4)
            {
                DaemonReload(go_motor_instance[i]->daemon);
                go_motor_instance[i]->dt = DWT_GetDeltaT(&go_motor_instance[i]->feed_cnt);
                if((_instance->recv_buff[14]<<8 | _instance->recv_buff[15]) == crc_ccitt(_instance->recv_buff, 14))
                {
                    go_motor_instance[i]->mes.tqr_fb = _instance->recv_buff[3]<<8 | _instance->recv_buff[4];
                    go_motor_instance[i]->mes.rot_spd = _instance->recv_buff[5]<<8 | _instance->recv_buff[6];
                    go_motor_instance[i]->mes.pos = _instance->recv_buff[7]<<24 | _instance->recv_buff[8]<<16 | _instance->recv_buff[9]<<8 | _instance->recv_buff[10];
                    go_motor_instance[i]->mes.tem = _instance->recv_buff[11];
                    go_motor_instance[i]->mes.wrong = _instance->recv_buff[12]>>5;
                    go_motor_instance[i]->rs485_flag = 1;
                }
            }
            /* code */
        }
        
    }
}

static void GOMotorLostCallback(void *motor_ptr)
{

}

static void GOMotorPacker(GOMotorInstance* _instance)
{
    _instance->ctrl_send.head[0] = 0xfe;
    _instance->ctrl_send.head[1] = 0xee;
    _instance->ctrl_send.Set = _instance->ctrl_set.ID<<4 | _instance->ctrl_set.Mode<<1;
    _instance->ctrl_send.tqr_set = _instance->ctrl_set.tqr_ft * 256;
    _instance->ctrl_send.spd_set = _instance->ctrl_set.spd_des * 128 / PI;
    _instance->ctrl_send.pos_set = _instance->ctrl_set.pos_des * 16384 / PI;
    _instance->ctrl_send.Kpos = _instance->ctrl_set.Kp * 1280;
    _instance->ctrl_send.Kspd = _instance->ctrl_set.Kd * 1280;
    _instance->ctrl_send.crc = crc_ccitt(&_instance->ctrl_send, 14);
}

static void GOMotorSend(GOMotorInstance* _instance)
{
    GOMotorPacker(_instance);
    if(_instance->rs485_flag == 1)
    {
        USARTSend(_instance->motor_usart, (uint8_t*)&_instance->ctrl_send, 17, USART_TRANSFER_DMA);
        _instance->rs485_flag = 0;
    }
}


static void GOMotorEnable(GOMotorInstance* _instance)
{
    _instance->stop_flag = MOTOR_ENALBED;
    _instance->ctrl_set.Mode = foc;
    _instance->rs485_flag = 1;
    GOMotorSend(_instance);
}

GOMotorInstance *GOmotorInit(Other_Motor_Init_Config_s config)
{
    GOMotorInstance *instance = (GOMotorInstance*)malloc(sizeof(GOMotorInstance));
    memset(instance, 0, sizeof(GOMotorInstance));

    instance->ctrl_set.ID = config.conf.ID;
    instance->ctrl_set.Kp = config.conf.Kp;
    instance->ctrl_set.Kd = config.conf.Kd;
    instance->ctrl_set.Mode = config.conf.Mode;
    config.usart_init_config.module_callback = DecodeGOIMotor;
    config.usart_init_config.recv_buff_size = 8*sizeof(char);
    instance->motor_usart = USARTRegister(&config.usart_init_config);

    // 注册守护线程
    Daemon_Init_Config_s daemon_config = {
        .callback = GOMotorLostCallback,
        .owner_id = instance,
        .reload_count = 20, // 200ms未收到数据则丢失
    };
    instance->daemon = DaemonRegister(&daemon_config);

    GOMotorEnable(instance);
    go_motor_instance[idx++] = instance;
    return instance;
}

//@Todo: 目前只实现了力控，更多位控PID等请自行添加
void GOMotorTask(void const *argument)
{
    GOMotorInstance *motor = (GOMotorInstance *)argument;
    while (1)
    {
        go_motor_instance[0]->ctrl_set.tqr_ft = 1;
        GOMotorSend(motor);
        osDelay(2);
    }
}
void GOMotorControlInit()
{
    char go_task_name[5] = "go";
    // 遍历所有电机实例,创建任务
    if (!idx)
        return;
    for (size_t i = 0; i < idx; i++)
    {
        char go_id_buff[2] = {0};
        __itoa(i, go_id_buff, 10);
        strcat(go_task_name, go_id_buff);
        osThreadDef(go_task_name, GOMotorTask, osPriorityNormal, 0, 128);
        go_task_handle[i] = osThreadCreate(osThread(go_task_name), go_motor_instance[i]);
    }
}