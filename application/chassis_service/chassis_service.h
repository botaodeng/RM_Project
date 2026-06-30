#ifndef CHASSIS_SERVICE_H
#define CHASSIS_SERVICE_H


/**
 * @brief 机器人底盘其他控制任务初始化,会被RobotInit()调用
 * 
 */
void ChassisServiceInit();

/**
 * @brief 机器人底盘其他控制任务,200Hz频率运行
 * 
 */
void ChassisServiceTask();

/**
 * @brief 机器人底盘其他控制任务,高速任务,设计为1kHz频率运行
 * 
 */
void ChassisHighSpeedTask();

#endif // CHASSIS_SERVICE_H