#ifndef CAN_URGENT_C
#define CAN_URGENT_C

#include "can_urgent.h"

/**
 * @brief CANUrgent接收回调函数
 * 
 * @param instance CAN实例
 */
static void CANUrgentRxCallback(CANInstance *instance)
{
    CANUrgentInstance *ins = (CANUrgentInstance *)instance->id; // 注意写法,将can instance的id强制转换为CANUrgentInstance*类型

    if(ins->role == CANURGENT_ROLE_GIMBAL)
    {
        // 处理云台板接收到的数据
    }
    else if(ins->role == CANURGENT_ROLE_CHASSIS)
    {
        // 处理底盘板接收到的数据
    }
}

static void CANUrgentLostCallback(CANInstance *instance)
{
    CANUrgentInstance *ins = (CANUrgentInstance *)instance->id; // 注意写法,将can instance的id强制转换为CANUrgentInstance*类型
    ins->online = CANURGENT_STATE_OFFLINE;
}

CANUrgentInstance* CANUrgentInit(CAN_Urgent_Init_Config_s* config)
{
    CANUrgentInstance *instance = (CANUrgentInstance *)malloc(sizeof(CANUrgentInstance));
    memset(instance, 0, sizeof(CANUrgentInstance));

    instance->role = config->role;

    config->can_config.id = (void *)instance;
    config->can_config.can_module_callback = CANUrgentRxCallback;
    instance->can_ins = CANRegister(&config->can_config);

    Daemon_Init_Config_s daemon_config = {
        .callback = CANUrgentLostCallback,
        .owner_id = (void *)instance,
        .reload_count = config->deamon_count,
    };
    instance->daemon = DaemonRegister(&daemon_config);
    return instance;
}

void CANUrgentSend(CANUrgentInstance *instance, uint8_t *data)
{
    if(instance->role == CANURGENT_ROLE_GIMBAL)
    {
        // 发送云台板数据
    }
    else if(instance->role == CANURGENT_ROLE_CHASSIS)
    {
        // 发送底盘板数据
    }
}

void *CANUrgentGet(CANUrgentInstance *instance)
{
    instance->update_flag = 0;
    if(instance->role == CANURGENT_ROLE_GIMBAL)
    {
        instance->yaw_fb_last = instance->yaw_fb;
        return (void *)&instance->yaw_fb;
    }
    else if(instance->role == CANURGENT_ROLE_CHASSIS)
    {
        instance->yaw_cmd_last = instance->yaw_cmd;
        return (void *)&instance->yaw_cmd;
    }
    return NULL;
}

uint8_t CANUrgentIsOnline(CANUrgentInstance *instance)
{
    return instance->online == CANURGENT_STATE_ONLINE;
}
#endif // !CAN_URGENT_C