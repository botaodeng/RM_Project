#ifndef CAN_URGENT_C
#define CAN_URGENT_C

#include "can_urgent.h"
#include <stdlib.h>
#include <string.h>

#include "dmmotor.h"
#include "motor_def.h"

static CANUrgentInstance *urgent_ins = NULL; // 用于保存CANUrgent实例的指针,在CANUrgentTask中使用

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
        ins->yaw_fb.yaw_motor_angle_10000x = (int16_t)((uint8_t)data[0] | ((uint8_t)data[1]<<8));
        ins->yaw_fb.yaw_motor_speed_1000x = (int16_t)((uint8_t)data[2] | ((uint8_t)data[3]<<8));
        ins->yaw_fb.yaw_torque_1000x = (int16_t)((uint8_t)data[4] | ((uint8_t)data[5]<<8));
        ins->yaw_fb.error = data[6];
        ins->yaw_fb.seq = data[7];
    }
    else if(ins->role == CANURGENT_ROLE_CHASSIS)
    {
        // 处理底盘板接收到的数据
        ins->yaw_cmd_last = ins->yaw_cmd;
        uint8_t *data = (uint8_t *)instance->rx_buff;
        ins->yaw_cmd.yaw_angle_ref_10000x = (int16_t)((uint8_t)data[0] | ((uint8_t)data[1]<<8));
        ins->yaw_cmd.yaw_speed_ref_1000x = (int16_t)((uint8_t)data[2] | ((uint8_t)data[3]<<8));
        ins->yaw_cmd.yaw_torque_ref_1000x = (int16_t)((uint8_t)data[4] | ((uint8_t)data[5]<<8));
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
    urgent_ins = instance;
    return instance;
}

void CANUrgentSend(CANUrgentInstance *instance)
{
    if(instance->role == CANURGENT_ROLE_GIMBAL)
    {
        // 发送云台板数据
        uint8_t data[8];
        data[0] = (uint16_t)(instance->yaw_cmd.yaw_angle_ref_10000x) & 0xFF;
        data[1] = ((uint16_t)(instance->yaw_cmd.yaw_angle_ref_10000x) >> 8) & 0xFF;
        data[2] = (uint16_t)(instance->yaw_cmd.yaw_speed_ref_1000x) & 0xFF;
        data[3] = ((uint16_t)(instance->yaw_cmd.yaw_speed_ref_1000x) >> 8) & 0xFF;
        data[4] = (uint16_t)(instance->yaw_cmd.yaw_torque_ref_1000x) & 0xFF;
        data[5] = ((uint16_t)(instance->yaw_cmd.yaw_torque_ref_1000x) >> 8) & 0xFF;
        data[6] = instance->yaw_cmd.mode;
        data[7] = instance->yaw_cmd.seq;
        instance->yaw_cmd_last = instance->yaw_cmd;
        CANSetDLC(instance->can_ins, 8);
        memcpy(instance->can_ins->tx_buff, data, 8);
        CANTransmit(instance->can_ins,0.1f);
        
    }
    else if(instance->role == CANURGENT_ROLE_CHASSIS)
    {
        // 发送底盘板数据
        uint8_t data[8];
        data[0] = (uint16_t)(instance->yaw_fb.yaw_motor_angle_10000x) & 0xFF;
        data[1] = ((uint16_t)(instance->yaw_fb.yaw_motor_angle_10000x) >> 8) & 0xFF;
        data[2] = (uint16_t)(instance->yaw_fb.yaw_motor_speed_1000x) & 0xFF;
        data[3] = ((uint16_t)(instance->yaw_fb.yaw_motor_speed_1000x) >> 8) & 0xFF;
        data[4] = (uint16_t)(instance->yaw_fb.yaw_torque_1000x) & 0xFF;
        data[5] = ((uint16_t)(instance->yaw_fb.yaw_torque_1000x) >> 8) & 0xFF;
        data[6] = instance->yaw_fb.error;
        data[7] = instance->yaw_fb.seq;
        instance->yaw_fb_last = instance->yaw_fb;
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

void CANUrgentTask()
{
    if (urgent_ins == NULL)
    {
        return;
    }
    if(urgent_ins->role == CANURGENT_ROLE_GIMBAL)
    {
        DMMotorInstance *motor = dm_motor_instance[0]; // 云台电机是第一个电机实例,以后再做规范化设定
        Motor_Control_Setting_s *motor_setting; // 电机控制参数
        Motor_Controller_s *motor_controller;   // 电机控制器
        DM_Motor_Measure_s *measure = &motor->measure;            // 电机测量值
        float pid_measure, pid_ref;             // 电机PID测量值和设定值
        YawUrgentFeedback_s yaw_fb = urgent_ins->yaw_fb, yaw_fb_last = urgent_ins->yaw_fb;

        if (motor == NULL)
            return;
        motor_setting = &motor->motor_settings;
        motor_controller = &motor->motor_controller;
        measure->position = yaw_fb.yaw_motor_angle_10000x/10000.0f;
        measure->velocity = yaw_fb.yaw_motor_speed_1000x/1000.0f;
        measure->torque = yaw_fb.yaw_torque_1000x/1000.0f;
        measure->last_position = yaw_fb_last.yaw_motor_angle_10000x/10000.0f;
        pid_ref = motor_controller->pid_ref;

        // 防止给未使能的电机发送控制信号
        if(motor->enabled_flag == DM_DISABLED || motor->stop_flag == MOTOR_STOP)
            urgent_ins->yaw_cmd.mode = 0;
        else
        {
            urgent_ins->yaw_cmd.mode = 1;
        }
        
        // 控制计算
        if(motor_setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            pid_ref *= -1;
        
        // pid_ref会顺次通过被启用的闭环充当数据的载体
        // 计算位置环,只有启用位置环且外层闭环为位置时会计算速度环输出
        if ((motor_setting->close_loop_type & ANGLE_LOOP) && motor_setting->outer_loop_type == ANGLE_LOOP)
        {
            if (motor_setting->angle_feedback_source == OTHER_FEED)
                pid_measure = *motor_controller->other_angle_feedback_ptr;
            else
                pid_measure = measure->position; // MOTOR_FEED,对total angle闭环,防止在边界处出现突跃
            // 更新pid_ref进入下一个环
            pid_ref = PIDCalculate(&motor_controller->angle_PID, pid_measure, pid_ref);
        }

        // 计算速度环,(外层闭环为速度或位置)且(启用速度环)时会计算速度环
        if ((motor_setting->close_loop_type & SPEED_LOOP) && (motor_setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)))
        {
            if (motor_setting->feedforward_flag & SPEED_FEEDFORWARD)
                pid_ref += *motor_controller->speed_feedforward_ptr;

            if (motor_setting->speed_feedback_source == OTHER_FEED)
                pid_measure = *motor_controller->other_speed_feedback_ptr;
            else // MOTOR_FEED
                pid_measure = measure->velocity;
            // 更新pid_ref进入下一个环
            pid_ref = PIDCalculate(&motor_controller->speed_PID, pid_measure, pid_ref);
        }

        if(motor_setting->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE)
            pid_ref *= -1;
        
        // 如果电机处于停止状态,将pid_ref置为0
        if(motor->stop_flag == MOTOR_STOP)
        {
            pid_ref = 0;
        }
        // 云台板发送数据
        urgent_ins->yaw_cmd.yaw_torque_ref_1000x = pid_ref*1000;
        CANUrgentSend(urgent_ins);
    }
    else if(urgent_ins->role == CANURGENT_ROLE_CHASSIS)
    {
        static uint16_t enable_send_cnt = 0;
        // 底盘板发送数据
        DM_Motor_Measure_s measure = dm_motor_instance[0]->measure;
        YawUrgentFeedback_s yaw_fb;
        YawUrgentCmd_s yaw_cmd;
        CANInstance *can_ins = dm_motor_instance[0]->motor_can_instace;
        uint8_t mode = urgent_ins->yaw_cmd.mode;
        urgent_ins->yaw_cmd = yaw_cmd;
        if(mode == 1)
            DMMotorMITSend(can_ins,0.0f,0.0f,0.0f,0.0f,urgent_ins->yaw_cmd.yaw_torque_ref_1000x/1000.0f);
        else if(mode == 0)
            DMMotorMITSend(can_ins,0.0f,0.0f,0.0f,0.0f,0.0f);
        yaw_fb.yaw_motor_angle_10000x = measure.position*10000;
        yaw_fb.yaw_motor_speed_1000x = measure.velocity*1000;
        yaw_fb.yaw_torque_1000x = measure.torque*1000;
        urgent_ins->yaw_fb = yaw_fb;
        CANUrgentSend(urgent_ins);
        if (++enable_send_cnt >= 500)
        {
            enable_send_cnt = 0;
            DMMotorEnable(dm_motor_instance[0]);
        }
    }
    else{
        return;
    }
}
#endif // !CAN_URGENT_C