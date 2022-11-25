/**
 * @file can_config.h
 * @brief CAN引脚配置
 * @author wangh (wanghuan3037@fiberhome.com)
 * @version 1.0
 * @date 2022-03-23
 * @copyright Copyright (c) 2022
 */
#ifndef __CAN_CONFIG_H
#define __CAN_CONFIG_H 
#include <rtthread.h>

#if defined(BSP_USING_CAN0)
#ifndef CAN0_CONFIG
#define CAN0_CONFIG               \
    {                             \
        .name = "can0",           \
        .can_periph = CAN0,       \
        .per_clk = RCU_CAN0,      \
        .tx_pin = GET_PIN(A, 12), \
        .rx_pin = GET_PIN(A, 11), \
        .remap = 0,               \
    }
#endif
#endif /* CAN0_CONFIG */

#if defined(BSP_USING_CAN1)
#ifndef CAN1_CONFIG
#define CAN1_CONFIG               \
    {                             \
        .name = "can1",           \
        .can_periph = CAN1,       \
        .per_clk = RCU_CAN1,      \
        .tx_pin = GET_PIN(D, 6),  \
        .rx_pin = GET_PIN(D, 5),  \
        .remap = GPIO_CAN1_REMAP, \
    }
#endif
#endif /* CAN0_CONFIG */

#endif	 // __CAN_CONFIG_H 

/*********** (C) COPYRIGHT 2022 FiberHome *****END OF FILE****/
