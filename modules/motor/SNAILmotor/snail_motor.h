/**
 * @file snail_motor.h
 * @author alex
 * @brief snail motor头文件
 * @version 0.1
 * @date 2025-05-22
 *
 * @todo  

 * @copyright Copyright (c) 2025 UCSB Gauchos all rights reserved
 *
 */

#ifndef SNAIL_MOTOR_H
#define SNAIL_MOTOR_H

#include "bsp_pwm.h"
#include "controller.h"
#include "motor_def.h"
#include "stdint.h"
#include "daemon.h"


/**
 * @brief snail motor typedef
 *
 */
typedef struct
{

    PWMInstance *motor_pwm_instance; // 电机PWM实例

    Motor_Type_e motor_type;        // 电机类型
    Motor_Working_Type_e stop_flag; // 启停标志

    DaemonInstance* daemon;
    uint32_t feed_cnt;
    float dt;
} SNAILMotorInstance;

/**
 * @brief 调用此函数注册一个snail电机,需要传递较多的初始化参数,请在application初始化的时候调用此函数
 *        推荐传参时像标准库一样构造initStructure然后传入此函数.
 *        recommend: type xxxinitStructure = {.ember1=xx,
 *                                            .ember2=xx,
 *                                             ....};
 *
 * @attention 这是PWM驱动的电机
 *
 * @param config 电机初始化结构体,包含了电机控制设置,电机PID参数设置,电机类型以及电机挂载的PWM设置
 *
 * @return DJIMotorInstance*
 */
SNAILMotorInstance *SNAILMotorInit(PWM_Motor_Init_Config_s *config);

/**
 * @brief 被application层的应用调用,给电机设定参考值.
 * @note 该函数会间接设置PWM占空比,如果电机处于停止状态,则会将占空比设置为0
 * @param motor 要设置的电机
 * @param ref 设定参考值
 */
void SNAILMotorSetRef(SNAILMotorInstance *motor, float ref);

/**
 * @brief 被application层的应用调用,给电机设定参考值.
 * @note 该函数会ramp设置PWM占空比,如果电机处于停止状态,则会将占空比设置为0
 * @param motor 要设置的电机
 * @param ref 设定参考值
 */
void SNAIL_motor_ramp(SNAILMotorInstance *motor, float ref);

/**
 * @brief 该函数被motor_task调用运行在rtos上,motor_stask内通过osDelay()确定控制频率
 */
void SNAILMotorControl();

/**
 * @brief 停止电机
 *
 */
void SNAILMotorStop(SNAILMotorInstance *motor);

/**
 * @brief 启动电机,此时电机会响应设定值
 *        初始化时不需要此函数,因为stop_flag的默认值为0
 *
 */
void SNAILMotorEnable(SNAILMotorInstance *motor);

#endif // SNAIL_MOTOR_H