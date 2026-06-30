#ifndef CHASSIS_SERVICE_C
#define CHASSIS_SERVICE_C

#include "robot_def.h"
#include "robot_cmd.h"
#include "chassis_service.h"

#include "dji_motor.h"
#include "dmmotor.h"
#include "can_comm.h"
#include "can_urgent.h"

#include "bsp_can.h"
// yaw电机实例,用于控制yaw电机
static DMMotorInstance *yaw;

static YawUrgentCmd_s yaw_cmd_recv;
static YawUrgentFeedback_s yaw_feedback_data;

// 拨盘电机实例,用于控制拨盘电机
static DJIMotorInstance *loader;

static Chassis_Ctrl_Cmd_s chassis_cmd_recv;


void ChassisServiceInit()
{
    // 拨盘电机
    Motor_Init_Config_s loader_config = {
        .can_init_config = {
            .can_handle = &hcan3,
            .tx_id = 5,
        },
        .controller_param_init_config = {
            .angle_PID = {
                // 如果启用位置环来控制发弹,需要较大的I值保证输出力矩的线性度否则出现接近拨出的力矩大幅下降
                .Kp = 0, // 10
                .Ki = 0,
                .Kd = 0,
                .MaxOut = 200,
            },
            .speed_PID = {
                .Kp = 1, // 10
                .Ki = 0, // 1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000,
                .MaxOut = 5000,
            },
            .current_PID = {
                .Kp = 0.7, // 0.7
                .Ki = 0, // 0.1
                .Kd = 0,
                .Improve = PID_Integral_Limit,
                .IntegralLimit = 5000,
                .MaxOut = 5000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED, .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP, // 初始化成SPEED_LOOP,让拨盘停在原地,防止拨盘上电时乱转
            .close_loop_type = CURRENT_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL, // 注意方向设置为拨盘的拨出的击发方向
        },
        .motor_type = M2006 // 英雄使用m3508
    };
    loader = DJIMotorInit(&loader_config);

    // yaw电机

}

void ChassisServiceTask()
{
    chassis_cmd_recv = *(Chassis_Ctrl_Cmd_s *)CANCommGet(cmd_can_comm);

    if(chassis_cmd_recv.friction_mode == FRICTION_OFF)
    {
        // 摩擦轮没开，不动拨盘
        DJIMotorStop(loader);
        return;
    }
    else
    {
        DJIMotorEnable(loader);
    }

    switch(chassis_cmd_recv.loader_mode)
    {
    // 停止拨盘
    case LOAD_STOP:
        DJIMotorOuterLoop(loader, SPEED_LOOP); // 切换到速度环
        DJIMotorSetRef(loader, 0);             // 同时设定参考值为0,这样停止的速度最快
        break;
    // 单发模式,根据鼠标按下的时间,触发一次之后需要进入不响应输入的状态(否则按下的时间内可能多次进入,导致多次发射)
    case LOAD_1_BULLET:                                                                     // 激活能量机关/干扰对方用,英雄用.
        DJIMotorOuterLoop(loader, ANGLE_LOOP);                                              // 切换到角度环
        DJIMotorSetRef(loader, loader->measure.total_angle + ONE_BULLET_DELTA_ANGLE); // 控制量增加一发弹丸的角度
        /*
        hibernate_time = DWT_GetTimeline_ms();                                              // 记录触发指令的时间
        dead_time = 150;                                                                    // 完成1发弹丸发射的时间
        break;
        */
    // 三连发,如果不需要后续可能删除
    case LOAD_3_BULLET:
        DJIMotorOuterLoop(loader, ANGLE_LOOP);                                                  // 切换到速度环
        DJIMotorSetRef(loader, loader->measure.total_angle + 3 * ONE_BULLET_DELTA_ANGLE); // 增加3发
        /*
        hibernate_time = DWT_GetTimeline_ms();                                                  // 记录触发指令的时间
        dead_time = 300;                                                                        // 完成3发弹丸发射的时间
        */
        break;
    // 连发模式,对速度闭环,射频后续修改为可变,目前固定为1Hz
    case LOAD_BURSTFIRE:
        DJIMotorOuterLoop(loader, SPEED_LOOP);
        DJIMotorSetRef(loader, chassis_cmd_recv.shoot_rate * 360 * REDUCTION_RATIO_LOADER / 8);
        // x颗/秒换算成速度: 已知一圈的载弹量,由此计算出1s需要转的角度,注意换算角速度(DJIMotor的速度单位是angle per second)
        break;
    // 拨盘反转,对速度闭环,后续增加卡弹检测(通过裁判系统剩余热量反馈和电机电流)
    // 也有可能需要从switch-case中独立出来
    case LOAD_REVERSE:
        DJIMotorOuterLoop(loader, SPEED_LOOP);
        // ...
        break;
    default:
        while (1)
            ; // 未知模式,停止运行,检查指针越界,内存溢出等问题
    }

}

void ChassisHighSpeedTask()
{
    if(cmd_can_urgent->role == CANURGENT_ROLE_GIMBAL)
    {
        // 处理云台板接收到的数据
        yaw_feedback_data = *(YawUrgentFeedback_s *)CANUrgentGet(cmd_can_urgent);

    }
    else if(cmd_can_urgent->role == CANURGENT_ROLE_CHASSIS)
    {
        // 处理底盘板接收到的数据
        yaw_cmd_recv = *(YawUrgentCmd_s *)CANUrgentGet(cmd_can_urgent);
    }
    else
    {
        // 未知角色
        cmd_can_urgent->online = CANURGENT_STATE_ERROR;
    }
}

#endif // CHASSIS_SERVICE_C