#ifndef CAN_URGENT_C
#define CAN_URGENT_C

#include "can_urgent.h"
#include <stdlib.h>
#include <string.h>

/**
 * @brief CANUrgent接收回调函数
 * 
 * @param instance CAN实例
 */
static void CANUrgentRxCallback(CANInstance *instance)
{
    CANUrgentInstance *ins = (CANUrgentInstance *)instance->id; // 注意写法,将can instance的id强制转换为CANUrgentInstance*类型

    if(instance->rx_len !=8)
    {
        ins->online = CANURGENT_STATE_ERROR;
        return;
    }
    if(ins->role == CANURGENT_ROLE_GIMBAL)
    {
        // 处理云台板接收到的数据
        ins->yaw_fb_last = ins->yaw_fb;
        uint8_t *data = (uint8_t *)instance->rx_buff;
        ins->yaw_fb.yaw_motor_angle = (int16_t)((uint8_t)data[0] | ((uint8_t)data[1]<<8));
        ins->yaw_fb.yaw_motor_speed = (int16_t)((uint8_t)data[2] | ((uint8_t)data[3]<<8));
        ins->yaw_fb.yaw_torque = (int16_t)((uint8_t)data[4] | ((uint8_t)data[5]<<8));
        ins->yaw_fb.error = data[6];
        ins->yaw_fb.seq = data[7];
    }
    else if(ins->role == CANURGENT_ROLE_CHASSIS)
    {
        // 处理底盘板接收到的数据
        ins->yaw_cmd_last = ins->yaw_cmd;
        uint8_t *data = (uint8_t *)instance->rx_buff;
        ins->yaw_cmd.yaw_angle_ref = (int16_t)((uint8_t)data[0] | ((uint8_t)data[1]<<8));
        ins->yaw_cmd.yaw_speed_ref = (int16_t)((uint8_t)data[2] | ((uint8_t)data[3]<<8));
        ins->yaw_cmd.yaw_torque_ref = (int16_t)((uint8_t)data[4] | ((uint8_t)data[5]<<8));
        ins->yaw_cmd.mode = data[6];
        ins->yaw_cmd.seq = data[7];
    }
    else
    {
        // 未知角色
        ins->online = CANURGENT_STATE_ERROR;
        return;
    }
    ins->online = CANURGENT_STATE_ONLINE;
    ins->update_flag = 1;
    if (ins->daemon != NULL)
    {
        DaemonReload(ins->daemon);
    }
}

static void CANUrgentLostCallback(void *instance)
{
    CANUrgentInstance *ins = (CANUrgentInstance *)instance;
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

void CANUrgentSend(CANUrgentInstance *instance)
{
    if(instance->role == CANURGENT_ROLE_GIMBAL)
    {
        // 发送云台板数据
        uint8_t data[8];
        data[0] = (uint16_t)(instance->yaw_cmd.yaw_angle_ref) & 0xFF;
        data[1] = ((uint16_t)(instance->yaw_cmd.yaw_angle_ref) >> 8) & 0xFF;
        data[2] = (uint16_t)(instance->yaw_cmd.yaw_speed_ref) & 0xFF;
        data[3] = ((uint16_t)(instance->yaw_cmd.yaw_speed_ref) >> 8) & 0xFF;
        data[4] = (uint16_t)(instance->yaw_cmd.yaw_torque_ref) & 0xFF;
        data[5] = ((uint16_t)(instance->yaw_cmd.yaw_torque_ref) >> 8) & 0xFF;
        data[6] = instance->yaw_cmd.mode;
        data[7] = instance->yaw_cmd.seq;
        CANSetDLC(instance->can_ins, 8);
        memcpy(instance->can_ins->tx_buff, data, 8);
        CANTransmit(instance->can_ins,0.1f);
        
    }
    else if(instance->role == CANURGENT_ROLE_CHASSIS)
    {
        // 发送底盘板数据
        uint8_t data[8];
        data[0] = (uint16_t)(instance->yaw_fb.yaw_motor_angle) & 0xFF;
        data[1] = ((uint16_t)(instance->yaw_fb.yaw_motor_angle) >> 8) & 0xFF;
        data[2] = (uint16_t)(instance->yaw_fb.yaw_motor_speed) & 0xFF;
        data[3] = ((uint16_t)(instance->yaw_fb.yaw_motor_speed) >> 8) & 0xFF;
        data[4] = (uint16_t)(instance->yaw_fb.yaw_torque) & 0xFF;
        data[5] = ((uint16_t)(instance->yaw_fb.yaw_torque) >> 8) & 0xFF;
        data[6] = instance->yaw_fb.error;
        data[7] = instance->yaw_fb.seq;
        CANSetDLC(instance->can_ins, 8);
        memcpy(instance->can_ins->tx_buff, data, 8);
        CANTransmit(instance->can_ins,0.1f);
    }
    else
    {
        // 未知角色
        instance->online = CANURGENT_STATE_ERROR;
        return;
    }
}

void *CANUrgentGet(CANUrgentInstance *instance)
{
    instance->update_flag = 0;
    if(instance->role == CANURGENT_ROLE_GIMBAL)
    {
        return (void *)&instance->yaw_fb;
    }
    else if(instance->role == CANURGENT_ROLE_CHASSIS)
    {
        return (void *)&instance->yaw_cmd;
    }
    return NULL;
}

uint8_t CANUrgentIsOnline(CANUrgentInstance *instance)
{
    if (instance == NULL || instance->daemon == NULL)
        return 0;

    return DaemonIsOnline(instance->daemon);
}
#endif // !CAN_URGENT_C