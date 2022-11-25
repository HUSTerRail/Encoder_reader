/**
 * @file drv_can.h
 * @brief GD32 CAN-RTT适配驱动
 * @author wangh (wanghuan3037@fiberhome.com)
 * @version 1.0
 * @date 2022-03-23
 * @copyright Copyright (c) 2022  烽火通信
 */
#ifndef __DRV_CAN_H
#define __DRV_CAN_H 
#include <board.h>
#include "can_config.h"

typedef struct 
{
    const char *name;
    uint32_t can_periph;
    rcu_periph_enum per_clk;
    uint32_t tx_pin;
    uint32_t rx_pin;
    uint32_t remap;
} gd32_can_config;


typedef struct
{
    gd32_can_config *config;
    struct rt_can_device device;
} gd32_can_device;




#endif	 // __DRV_CAN_H 

/*********** (C) COPYRIGHT 2022 FiberHome *****END OF FILE****/
