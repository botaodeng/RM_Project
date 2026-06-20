#ifndef DMMOTOR_H
#define DMMOTOR_H
#include <stdint.h>
#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "daemon.h"

#ifndef DM_CONSTRAIN
#define DM_CONSTRAIN
#pragma message "check MIN, MAX values for DM_P, DM_V, DM_T"
#endif
#define DM_MOTOR_CNT 4

#define DM_P_MIN  (-12.5f)
#define DM_P_MAX  12.5f
#define DM_V_MIN  (-45.0f)
#define DM_V_MAX  45.0f
#define DM_T_MIN  (-18.0f)
#define DM_T_MAX   18.0f
#define DM_Kp_MIN  (0.0f)
#define DM_Kp_MAX  (500.0f)
#define DM_Kd_MIN  (0.0f)
#define DM_Kd_MAX  (5.0f)

/* 电机测量值结构体,保存电机的反馈值 */
typedef struct 
{
    uint8_t id;
    uint8_t state;
    float velocity;
    float last_position;
    float position;
    float torque;
    float T_Mos;
    float T_Rotor;
    int32_t total_round;
}DM_Motor_Measure_s;

/* 电机发送报文结构体,保存要发送的控制量 */
typedef struct
{
    uint16_t position_des; // 位置
    uint16_t velocity_des; // 速度
    uint16_t torque_des;   // 力矩
    uint16_t Kp;           // 
    uint16_t Kd;
}DMMotor_Send_s;

typedef enum
{
    DM_ENABLED = 0,
    DM_DISABLED = 1,
}DMMotor_Enabled_e;

/* 电机实例结构体,保存电机的测量值,控制设置,PID实例,CAN实例等 */
typedef struct 
{
    DM_Motor_Measure_s measure;                // 电机测量值
    Motor_Control_Setting_s motor_settings;    // 电机设置
    Motor_Controller_s motor_controller;       // 电机控制器

    /*
    PIDInstance current_PID;
    PIDInstance speed_PID;
    PIDInstance angle_PID;
    float *other_angle_feedback_ptr;
    float *other_speed_feedback_ptr;
    float *speed_feedforward_ptr;
    float *current_feedforward_ptr;
    float pid_ref;
    */

    CANInstance *motor_can_instace;            // 电机can实例

    Motor_Type_e motor_type;                   // 电机类型
    Motor_Working_Type_e stop_flag;            // 电机启停标志
    DMMotor_Enabled_e enabled_flag;            // 电机使能标志

    DaemonInstance* motor_daemon;
    uint32_t lost_cnt;

}DMMotorInstance;

typedef enum
{
    DM_CMD_MOTOR_MODE = 0xfc,   // 使能,会响应指令
    DM_CMD_RESET_MODE = 0xfd,   // 停止
    DM_CMD_ZERO_POSITION = 0xfe, // 将当前的位置设置为编码器零位
    DM_CMD_CLEAR_ERROR = 0xfb // 清除电机过热错误
}DMMotor_Mode_e;

/**
 * @brief 调用此函数注册一个DM电机,需要传递较多的初始化参数,请在application初始化的时候调用此函数
 *        推荐传参时像标准库一样构造initStructure然后传入此函数.
 *        recommend: type xxxinitStructure = {.member1=xx,
 *                                            .member2=xx,
 *                                             ....};
 *
 * @attention 注意电机id
 *
 * @param config 电机初始化结构体,包含了电机控制设置,电机PID参数设置,电机类型以及电机挂载的CAN设置
 *
 * @return DMMotorInstance*
 */
DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config);

/**
 * @brief 被application层的应用调用,给电机设定参考值.
 *        对于应用,可以将电机视为传递函数为1的设备,不需要关心底层的闭环
 *
 * @param motor 要设置的电机
 * @param ref 设定参考值
 */
void DMMotorSetRef(DMMotorInstance *motor, float ref);

/**
 * @brief 该函数被motor_task调用运行在rtos上,motor_stask内通过osDelay()确定控制频率
 */
void DMMotorControl();

/**
 * @brief 修改电机闭环目标(外层闭环)
 *
 * @param motor  要修改的电机实例指针
 * @param outer_loop 外层闭环类型
 */
void DMMotorOuterLoop(DMMotorInstance *motor,Closeloop_Type_e closeloop_type);

/**
 * @brief 使电机进入使能状态
 *
 */
void DMMotorEnable(DMMotorInstance *motor);

/**
 * @brief 使电机进入失能
 *
 */
void DMMotorDisable(DMMotorInstance *motor);

/**
 * @brief 停止电机，注意不是将设定值设为零,而是直接给电机发送的电流值置零
 */
void DMMotorStop(DMMotorInstance *motor);

/**
 * @brief 使电机重新校准编码器零位,调用此函数后电机会将当前的位置设置为编码器零位
 * 
 * @attention 此函数会发送一个特殊的控制指令,电机收到后会立刻将当前的位置设置为编码器零位,请确保在安全的情况下调用此函数,请不要随意调用此函数
 *
 */
void DMMotorCaliEncoder(DMMotorInstance *motor);
/**
 * @brief 通过MIT模式发送电机控制指令
 *
 * @param motor 要发送控制指令的电机实例
 * @param p_des 期望位置
 * @param v_des 期望速度
 * @param kp 位置环比例增益
 * @param kd 速度环比例增益
 * @param t_ff 力矩
 */
void DMMotorMITSend(CANInstance *can_instance, float p_des, float v_des, float kp, float kd, float t_ff);

/*
void DMMotorControlInit();
*/
#endif // !DMMOTOR