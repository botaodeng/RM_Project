#ifndef CAN_URGENT_H
#define CAN_URGENT_H

#include "bsp_can.h"
#include "daemon.h"
#include <stdint.h>

typedef enum {
    CANURGENT_ROLE_GIMBAL = 0,
    CANURGENT_ROLE_CHASSIS,
} CANUrgentRole_e;

typedef enum {
    CANURGENT_STATE_OFFLINE = 0,
    CANURGENT_STATE_ONLINE,
    CANURGENT_STATE_ERROR,
} CANUrgentState_e;

/* 云台板 -> 底盘板 指令结构体*/
typedef struct __attribute__((packed)) {
    int16_t yaw_angle_ref;
    int16_t yaw_speed_ref;
    int16_t yaw_torque_ref;
    uint8_t mode;
    uint8_t seq;
} YawUrgentCmd_s; // 8 bytes

/* 底盘板 -> 云台板 反馈结构体*/
typedef struct __attribute__((packed)) {
    int16_t yaw_motor_angle;
    int16_t yaw_motor_speed;
    int16_t yaw_torque;  //DM4310回传torque
    uint8_t error;
    uint8_t seq;
} YawUrgentFeedback_s; // 8 bytes

/* CAN Urgent 初始化结构体*/
typedef struct {
    CAN_Init_Config_s can_config; //CAN初始化结构体
    CANUrgentRole_e role; //角色

    uint16_t deamon_count; //守护进程计数
}CAN_Urgent_Init_Config_s;

/* CAN Urgent 实例结构体*/
typedef struct {
    CANInstance *can_ins;
    CANUrgentRole_e role;

    YawUrgentCmd_s yaw_cmd;
    YawUrgentCmd_s yaw_cmd_last;

    YawUrgentFeedback_s yaw_fb;
    YawUrgentFeedback_s yaw_fb_last;

    uint8_t tx_seq;
    uint8_t update_flag;
    CANUrgentState_e online; //在线状态

    DaemonInstance *daemon;
} CANUrgentInstance;

/**
 * @brief 初始化CANUrgent实例
 *
 * @param config CANUrgent初始化结构体
 * @return CANUrgentInstance*
 */
CANUrgentInstance* CANUrgentInit(CAN_Urgent_Init_Config_s* config);

/**
 * @brief 通过CANUrgent发送数据
 *
 * @param instance canurgent实例
 */
void CANUrgentSend(CANUrgentInstance *instance);

/**
 * @brief 获取CANUrgent接收的数据,需要自己使用强制类型转换将返回的void指针转换成指定类型
 *
 * @return void* 返回的数据指针
 */
void *CANUrgentGet(CANUrgentInstance *instance);

/**
 * @brief 检查CANUrgent是否在线
 * 
 * @param instance 
 * @return uint8_t 
 */
uint8_t CANUrgentIsOnline(CANUrgentInstance *instance);

/**
 * @brief CANUrgent任务函数,在motor task中调用,用于处理CANUrgent的发送和接收，并且实现双板控制电机
 */
void CANUrgentTask();
#endif // !CAN_URGENT_H
