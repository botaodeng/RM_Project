#include "remote_control.h"
#include "string.h"
#include "bsp_usart.h"
#include "memory.h"
#include "stdlib.h"
#include "daemon.h"
#include "bsp_log.h"
#include "crc_ref.h"

#define REMOTE_CONTROL_FRAME_SIZE 21u // 遥控器接收的buffer大小
#define CRC_INIT 0xFFFF
static uint8_t rc_recv_buff[REMOTE_CONTROL_FRAME_SIZE];
// 遥控器数据
static RC_ctrl_t rc_ctrl[2];     //[0]:当前数据TEMP,[1]:上一次的数据LAST.用于按键持续按下和切换的判断
static uint8_t rc_init_flag = 0; // 遥控器初始化标志位
static uint16_t wcrc = 0;
// 遥控器拥有的串口实例,因为遥控器是单例,所以这里只有一个,就不封装了
static USARTInstance *rc_usart_instance;
static DaemonInstance *rc_daemon_instance;

/**
 * @brief 矫正遥控器摇杆的值,超过800或者小于-800的值都认为是无效值,置0
 *
 */
static void RectifyRCjoystick()
{
    for (uint8_t i = 0; i < 5; ++i)
        if (abs(*(&rc_ctrl[TEMP].rocker_l_ + i)) > 700)
            *(&rc_ctrl[TEMP].rocker_l_ + i) = 0;
}



/**
 * @brief 遥控器数据解析
 *
 * @param sbus_buf 接收buffer
 */
static void sbus_to_rc(const uint8_t *sbus_buf)
{
    if(sbus_buf == NULL)
    {
        rc_ctrl[TEMP].online_flag = 0;
        return;
    }

    uint16_t head = (sbus_buf[0] | (sbus_buf[1] << 8));
    if(head != 0x53A9){
        rc_ctrl[TEMP].online_flag = 0;
        return;
    }

    uint16_t crc = (sbus_buf[REMOTE_CONTROL_FRAME_SIZE - 2] | (sbus_buf[REMOTE_CONTROL_FRAME_SIZE - 1] << 8));
    for(int i =0; i < (REMOTE_CONTROL_FRAME_SIZE - 2); ++i)
        rc_recv_buff[i] = sbus_buf[i];
    if ((Get_CRC16_Check_Sum(rc_recv_buff, REMOTE_CONTROL_FRAME_SIZE - 2, CRC_INIT) != crc))
    {
        return;
    }
    memcpy(&rc_ctrl[LAST], &rc_ctrl[TEMP], sizeof(RC_ctrl_t)); // 保存上一次的数据,用于按键持续按下和切换的判断

    VTM_data_t *vtm_data = (VTM_data_t *)sbus_buf;
    rc_ctrl[TEMP].rocker_r_ = (vtm_data->ch_0- 1024.0f) * (1.0f / 660.0f);
    rc_ctrl[TEMP].rocker_r1 = (vtm_data->ch_1- 1024.0f) * (1.0f / 660.0f);
    rc_ctrl[TEMP].rocker_l1 = (vtm_data->ch_2- 1024.0f) * (1.0f / 660.0f);
    rc_ctrl[TEMP].rocker_l_ = (vtm_data->ch_3- 1024.0f) * (1.0f / 660.0f);
    rc_ctrl[TEMP].mode_sw = vtm_data->mode_sw;
    rc_ctrl[TEMP].pause = vtm_data->pause;
    rc_ctrl[TEMP].fn_1 = vtm_data->fn_1;
    rc_ctrl[TEMP].fn_2 = vtm_data->fn_2;
    rc_ctrl[TEMP].wheel = (vtm_data->wheel - 1024.0f) * (1.0f / 660.0f);
    rc_ctrl[TEMP].trigger = vtm_data->trigger;

    rc_ctrl[TEMP].mouse_x = vtm_data->mouse_x;
    rc_ctrl[TEMP].mouse_y = vtm_data->mouse_y;
    rc_ctrl[TEMP].mouse_z = vtm_data->mouse_z;
    rc_ctrl[TEMP].mouse_left = vtm_data->mouse_left;
    rc_ctrl[TEMP].mouse_right = vtm_data->mouse_right;
    rc_ctrl[TEMP].mouse_middle = vtm_data->mouse_middle;
    rc_ctrl[TEMP].key.key_W = vtm_data->key & 0x01;
    rc_ctrl[TEMP].key.key_S = (vtm_data->key >> 1) & 0x01;
    rc_ctrl[TEMP].key.key_A = (vtm_data->key >> 2) & 0x01;
    rc_ctrl[TEMP].key.key_D = (vtm_data->key >> 3) & 0x01;
    rc_ctrl[TEMP].key.key_Shift = (vtm_data->key >> 4) & 0x01;
    rc_ctrl[TEMP].key.key_Ctrl = (vtm_data->key >> 5) & 0x01;
    rc_ctrl[TEMP].key.key_Q = (vtm_data->key >> 6) & 0x01;
    rc_ctrl[TEMP].key.key_E = (vtm_data->key >> 7) & 0x01;
    rc_ctrl[TEMP].key.key_R = (vtm_data->key >> 8) & 0x01;
    rc_ctrl[TEMP].key.key_F = (vtm_data->key >> 9) & 0x01;
    rc_ctrl[TEMP].key.key_G = (vtm_data->key >> 10) & 0x01;
    rc_ctrl[TEMP].key.key_Z = (vtm_data->key >> 11) & 0x01;
    rc_ctrl[TEMP].key.key_X = (vtm_data->key >> 12) & 0x01;
    rc_ctrl[TEMP].key.key_C = (vtm_data->key >> 13) & 0x01;
    rc_ctrl[TEMP].key.key_V = (vtm_data->key >> 14) & 0x01;
    rc_ctrl[TEMP].key.key_B = (vtm_data->key >> 15) & 0x01;

    rc_ctrl[TEMP].online_flag = 1;
}

/**
 * @brief 对sbus_to_rc的简单封装,用于注册到bsp_usart的回调函数中
 *
 */
static void RemoteControlRxCallback()
{
    DaemonReload(rc_daemon_instance);         // 先喂狗
    sbus_to_rc(rc_usart_instance->recv_buff); // 进行协议解析
}

/**
 * @brief 遥控器离线的回调函数,注册到守护进程中,串口掉线时调用
 *
 */
static void RCLostCallback(void *id)
{
    memset(rc_ctrl, 0, sizeof(rc_ctrl)); // 清空遥控器数据
    USARTServiceInit(rc_usart_instance); // 尝试重新启动接收
    LOGWARNING("[rc] remote control lost");
}

RC_ctrl_t *RemoteControlInit(UART_HandleTypeDef *rc_usart_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = RemoteControlRxCallback;
    conf.usart_handle = rc_usart_handle;
    conf.recv_buff_size = REMOTE_CONTROL_FRAME_SIZE;
    rc_usart_instance = USARTRegister(&conf);

    // 进行守护进程的注册,用于定时检查遥控器是否正常工作
    Daemon_Init_Config_s daemon_conf = {
        .reload_count = 10, // 100ms未收到数据视为离线,遥控器的接收频率实际上是1000/14Hz(大约70Hz)
        .callback = RCLostCallback,
        .owner_id = NULL, // 只有1个遥控器,不需要owner_id
    };
    rc_daemon_instance = DaemonRegister(&daemon_conf);

    rc_init_flag = 1;
    return rc_ctrl;
}

uint8_t RemoteControlIsOnline()
{
    if (rc_init_flag || rc_ctrl[TEMP].online_flag) // 遥控器尚未初始化或者接收数据出错都视为离线
        return DaemonIsOnline(rc_daemon_instance);
    return 0;
}