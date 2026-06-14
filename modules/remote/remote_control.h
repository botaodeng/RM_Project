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

// 用于遥控器数据读取,遥控器数据是一个大小为2的数组
#define LAST 1
#define TEMP 0

// 检查接收值是否出错
#define RC_CH_VALUE_MIN ((uint16_t) 0x00F0)
#define RC_CH_VALUE_OFFSET ((uint16_t) 0x0400)
#define RC_CH_VALUE_MAX ((uint16_t) 0x070F)

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

    int16_t rocker_l_; // 左水平
    int16_t rocker_l1; // 左竖直
    int16_t rocker_r_; // 右水平
    int16_t rocker_r1; // 右竖直

    int16_t vra;       // VRA
    int16_t vrb;       // VRB

    uint16_t switch_swa;  // 最左侧开关
    uint16_t switch_swb;  // 左侧开关
    uint16_t switch_swc;  // 右侧开关
    uint16_t switch_swd;  // 最右侧开关

    uint8_t online_flag; // 遥控器在线标志位,通过daemon定时检查遥控器是否在线,如果离线则置0
} RC_ctrl_t;

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
