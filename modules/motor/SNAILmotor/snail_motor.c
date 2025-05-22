#include "snail_motor.h"
#include "general_def.h"
#include "bsp_dwt.h"
#include "bsp_log.h"

static uint8_t idx = 0; // register idx,是该文件的全局电机索引,在注册时使用
/* SNAIL电机的实例,此处仅保存指针,内存的分配将通过电机实例初始化时通过malloc()进行 */
static SNAILMotorInstance *SNAIL_motor_instance

// 电机初始化,返回一个电机实例
SNAILMotorInstance *SNAILMotorInit(Motor_Init_Config_s *config)
{
    SNAILMotorInstance *instance = (SNAILMotorInstance *)malloc(sizeof(SNAILMotorInstance));
    memset(instance, 0, sizeof(SNAILMotorInstance));

    // motor basic setting 电机基本设置
    instance->motor_type = config->motor_type; 
    
    //注册电机到PWM
    instance->motor_pwm_instance = PWMRegister(&config->pwm_init_config);

    return instance;
}

void SNAILMotorStop(SNAILMotorInstance *motor)
{
    motor->stop_flag = MOTOR_STOP;
}

void SNAILMotorEnable(SNAILMotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}