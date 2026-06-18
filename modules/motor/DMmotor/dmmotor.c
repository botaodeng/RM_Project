#include "dmmotor.h"
#include "memory.h"
#include "general_def.h"
#include "user_lib.h"
#include "cmsis_os.h"
#include "string.h"
#include "daemon.h"
#include "stdlib.h"
#include "bsp_log.h"

static uint8_t idx=0; // register idx,是该文件的全局电机索引,在注册时使用
static DMMotorInstance *dm_motor_instance[DM_MOTOR_CNT];
static osThreadId dm_task_handle[DM_MOTOR_CNT];
/* 两个用于将uint值和float值进行映射的函数,在设定发送值和解析反馈值时使用 */
static uint16_t float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

static void DMMotorSetMode(DMMotor_Mode_e cmd, DMMotorInstance *motor)
{
    memset(motor->motor_can_instace->tx_buff, 0xff, 7);  // 发送电机指令的时候前面7bytes都是0xff
    motor->motor_can_instace->tx_buff[7] = (uint8_t)cmd; // 最后一位是命令id
    CANTransmit(motor->motor_can_instace, 1);
}

static void DMMotorDecode(CANInstance *motor_can)
{
    uint16_t tmp; // 用于暂存解析值,稍后转换成float数据,避免多次创建临时变量
    uint8_t *rxbuff = motor_can->rx_buff;
    DMMotorInstance *motor = (DMMotorInstance *)motor_can->id;
    DM_Motor_Measure_s *measure = &(motor->measure); // 将can实例中保存的id转换成电机实例的指针

    DaemonReload(motor->motor_daemon);

    measure->last_position = measure->position;
    tmp = (uint16_t)((rxbuff[1] << 8) | rxbuff[2]);
    measure->position = uint_to_float(tmp, DM_P_MIN, DM_P_MAX, 16);

    tmp = (uint16_t)((rxbuff[3] << 4) | rxbuff[4] >> 4);
    measure->velocity = uint_to_float(tmp, DM_V_MIN, DM_V_MAX, 12);

    tmp = (uint16_t)(((rxbuff[4] & 0x0f) << 8) | rxbuff[5]);
    measure->torque = uint_to_float(tmp, DM_T_MIN, DM_T_MAX, 12);

    measure->T_Mos = (float)rxbuff[6];
    measure->T_Rotor = (float)rxbuff[7];
}

static void DMMotorLostCallback(void *motor_ptr)
{
}

void DMMotorCaliEncoder(DMMotorInstance *motor)
{
    DMMotorSetMode(DM_CMD_ZERO_POSITION, motor);
}

DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config)
{
    DMMotorInstance *motor = (DMMotorInstance *)malloc(sizeof(DMMotorInstance));
    memset(motor, 0, sizeof(DMMotorInstance));

    // 基础设置
    motor->motor_type = config->motor_type;
    motor->motor_settings = config->controller_setting_init_config;


    // motor controller init 电机控制器初始化，无电流环，不配置
    PIDInit(&motor->motor_controller.speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&motor->motor_controller.angle_PID, &config->controller_param_init_config.angle_PID);
    motor->motor_controller.other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    motor->motor_controller.other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;
    motor->motor_controller.speed_feedforward_ptr = config->controller_param_init_config.speed_feedforward_ptr;

    // 注册电机到can总线
    config->can_init_config.can_module_callback = DMMotorDecode;
    config->can_init_config.id = motor;
    motor->motor_can_instace = CANRegister(&config->can_init_config);

    // 注册电机到守护进程
    Daemon_Init_Config_s conf = {
        .callback = DMMotorLostCallback,
        .owner_id = motor,
        .reload_count = 10,
    };
    motor->motor_daemon = DaemonRegister(&conf);

    DMMotorEnable(motor);

    dm_motor_instance[idx++] = motor;
    return motor;
}

void DMMotorSetRef(DMMotorInstance *motor, float ref)
{
    motor->motor_controller.pid_ref = ref;
}

void DMMotorEnable(DMMotorInstance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
    motor->enabled_flag = DM_ENABLED;
    DMMotorSetMode(DM_CMD_MOTOR_MODE, motor);
}

void DMMotorDisable(DMMotorInstance *motor)
{
    motor->stop_flag = MOTOR_STOP;
    motor->enabled_flag = DM_DISABLED;
    DMMotorSetMode(DM_CMD_RESET_MODE, motor);
}

void DMMotorOuterLoop(DMMotorInstance *motor, Closeloop_Type_e type)
{
    motor->motor_settings.outer_loop_type = type;
}


//@Todo: 目前只实现了力控，更多位控PID等请自行添加
//并非按照设定要求处理，重新按照djimotor处理
void DMMotorTask(void const *argument)
{
    float  pid_ref, set;
    DMMotorInstance *motor = (DMMotorInstance *)argument;
   //DM_Motor_Measure_s *measure = &motor->measure;
    Motor_Control_Setting_s *setting = &motor->motor_settings;
    //CANInstance *motor_can = motor->motor_can_instace;
    //uint16_t tmp;
    DMMotor_Send_s motor_send_mailbox;
    while (1)
    {
        pid_ref = motor->motor_controller.pid_ref;
        
        set = pid_ref;
        if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            set *= -1;
       
        LIMIT_MIN_MAX(set, DM_T_MIN, DM_T_MAX);
        motor_send_mailbox.position_des = float_to_uint(0, DM_P_MIN, DM_P_MAX, 16);
        motor_send_mailbox.velocity_des = float_to_uint(0, DM_V_MIN, DM_V_MAX, 12);
        motor_send_mailbox.torque_des = float_to_uint(pid_ref, DM_T_MIN, DM_T_MAX, 12);
        motor_send_mailbox.Kp = 0;
        motor_send_mailbox.Kd = 0;

        if(motor->stop_flag == MOTOR_STOP)
            motor_send_mailbox.torque_des = float_to_uint(0, DM_T_MIN, DM_T_MAX, 12);

        motor->motor_can_instace->tx_buff[0] = (uint8_t)(motor_send_mailbox.position_des >> 8);
        motor->motor_can_instace->tx_buff[1] = (uint8_t)(motor_send_mailbox.position_des);
        motor->motor_can_instace->tx_buff[2] = (uint8_t)(motor_send_mailbox.velocity_des >> 4);
        motor->motor_can_instace->tx_buff[3] = (uint8_t)(((motor_send_mailbox.velocity_des & 0xF) << 4) | (motor_send_mailbox.Kp >> 8));
        motor->motor_can_instace->tx_buff[4] = (uint8_t)(motor_send_mailbox.Kp);
        motor->motor_can_instace->tx_buff[5] = (uint8_t)(motor_send_mailbox.Kd >> 4);
        motor->motor_can_instace->tx_buff[6] = (uint8_t)(((motor_send_mailbox.Kd & 0xF) << 4) | (motor_send_mailbox.torque_des >> 8));
        motor->motor_can_instace->tx_buff[7] = (uint8_t)(motor_send_mailbox.torque_des);

        CANTransmit(motor->motor_can_instace, 1);

    }
}

/* 暂时不创建任务,由application层调用motor task中的函数进行控制 */
/*
void DMMotorControlInit()
{
    char dm_task_name[5] = "dm";
    // 遍历所有电机实例,创建任务
    if (!idx)

        return;
    for (size_t i = 0; i < idx; i++)
    {
        char dm_id_buff[2] = {0};
        __itoa(i, dm_id_buff, 10);
        strcat(dm_task_name, dm_id_buff);
        osThreadDef(dm_task_name, DMMotorTask, osPriorityNormal, 0, 128);
        dm_task_handle[i] = osThreadCreate(osThread(dm_task_name), dm_motor_instance[i]);
    }
}
*/

void DMMotorControl()
{
    int16_t set;        // 电机控制CAN发送设定值
    DMMotorInstance *motor;
    Motor_Control_Setting_s *motor_setting; // 电机控制参数
    Motor_Controller_s *motor_controller;   // 电机控制器
    DM_Motor_Measure_s *measure;           // 电机测量值
    float pid_measure, pid_ref;             // 电机PID测量值和设定值

    // 遍历所有电机实例,进行控制计算和CAN发送
    for (size_t i = 0; i < idx; i++)
    {
        motor = dm_motor_instance[i];
        motor_setting = &motor->motor_settings;
        motor_controller = &motor->motor_controller;
        measure = &motor->measure;
        pid_ref = motor_controller->pid_ref;

        // 防止给未使能的电机发送控制信号
        if(motor->enabled_flag == DM_DISABLED)
            continue;
        
        // 控制计算


        // CAN发送
        if(motor->stop_flag == MOTOR_STOP)
        {
            
        }
        CANTransmit(motor->motor_can_instace, 1);
    }
}

void DMMotorStop(DMMotorInstance *motor)
{
    motor->stop_flag = MOTOR_STOP;
}