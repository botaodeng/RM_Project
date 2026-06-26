#ifndef ROBOT_CMD_H
#define ROBOT_CMD_H

#ifdef CHASSIS_BOARD
#include "can_urgent.h"
#include "can_comm.h"
extern CANUrgentInstance *cmd_can_urgent; // 双板高速通信
extern CANCommInstance *cmd_can_comm; // 双板通信
#endif

/**
 * @brief 机器人核心控制任务初始化,会被RobotInit()调用
 * 
 */
void RobotCMDInit();

/**
 * @brief 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率)
 * 
 */
void RobotCMDTask();

#endif // !ROBOT_CMD_H