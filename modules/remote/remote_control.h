/**
 * @file remote_control.h
 * @author DJI 2016
 * @author modified by neozng
 * @brief  DR16/DT7遥控器模块定义头文件
 * @version beta
 * @date 2022-11-01
 *
 * @copyright Copyright (c) 2016 DJI corp
 * @copyright Copyright (c) 2022 HNU YueLu EC all rights reserved
 *
 */
#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include <stdint.h>
#include "main.h"
#include "usart.h"
#include "stdbool.h"

// 用于遥控器数据读取,遥控器数据是一个大小为2的数组
#define LAST 1
#define TEMP 0

/*
// 检查接收值是否出错
#define RC_CH_VALUE_MIN ((uint16_t) 0x00F0)
#define RC_CH_VALUE_OFFSET ((uint16_t) 0x0400)
#define RC_CH_VALUE_MAX ((uint16_t) 0x070F)
*/
/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP ((uint16_t) 0x00F0)   // 开关向上时的值
#define RC_SW_MID ((uint16_t) 0x0400)  // 开关中间时的值
#define RC_SW_DOWN ((uint16_t) 0x070F) // 开关向下时的值
// 三个判断开关状态的宏
#define switch_is_down(s) (s == RC_SW_DOWN)
#define switch_is_mid(s) (s == RC_SW_MID)
#define switch_is_up(s) (s == RC_SW_UP)



// @todo 当前结构体嵌套过深,需要进行优化
typedef struct
{
    bool key_W; // W键
    bool key_S; // S键
    bool key_A; // A键
    bool key_D; // D键
    bool key_Shift; // Shift键
    bool key_Ctrl; // Ctrl键
    bool key_Q; // Q键
    bool key_E; // E键
    bool key_R; // R键
    bool key_F; // F键
    bool key_G; // G键
    bool key_Z; // Z键
    bool key_X; // X键
    bool key_C; // C键
    bool key_V; // V键
    bool key_B; // B键
} KeyBoard_t;

typedef struct
{

    float rocker_l_; // 左水平
    float rocker_l1; // 左竖直
    float rocker_r_; // 右水平
    float rocker_r1; // 右竖直

    uint8_t mode_sw:2; // 模式开关
    uint8_t pause:1; // 暂停开关
    uint8_t fn_1:1;  // 功能键左
    uint8_t fn_2:1;  // 功能键右
    float wheel; // 拨轮
    uint8_t trigger:1; // 扳机

    int16_t mouse_x; // 鼠标X轴
    int16_t mouse_y; // 鼠标Y轴
    int16_t mouse_z; // 鼠标滚轮
    uint8_t mouse_left:2;   // 鼠标左键
    uint8_t mouse_right:2;  // 鼠标右键
    uint8_t mouse_middle:2; // 鼠标中键
    KeyBoard_t key; // 键盘按键

    uint8_t online_flag; // 遥控器在线标志位,通过daemon定时检查遥控器是否在线,如果离线则置0
} RC_ctrl_t;

typedef struct __attribute__((packed)){
    uint8_t sof_1;
    uint8_t sof_2;
    uint64_t ch_0:11;
    uint64_t ch_1:11;
    uint64_t ch_2:11;
    uint64_t ch_3:11;
    uint64_t mode_sw:2;
    uint64_t pause:1;
    uint64_t fn_1:1;
    uint64_t fn_2:1;
    uint64_t wheel:11;
    uint64_t trigger:1;

    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    uint8_t mouse_left:2;
    uint8_t mouse_right:2;
    uint8_t mouse_middle:2;
    uint16_t key;
    uint16_t crc16;
}VTM_data_t;

/* ------------------------- Internal Data ----------------------------------- */

/**
 * @brief 初始化遥控器,该函数会将遥控器注册到串口
 *
 * @attention 注意分配正确的串口硬件,遥控器在C板上使用USART3
 *
 */
RC_ctrl_t *RemoteControlInit(UART_HandleTypeDef *rc_usart_handle);

/**
 * @brief 检查遥控器是否在线,若尚未初始化也视为离线
 *
 * @return uint8_t 1:在线 0:离线
 */
uint8_t RemoteControlIsOnline();

#endif
