/**
 * @file drv_common.h
 * @brief gd32适配rtthread驱动
 * @author wangh (wanghuan3037@fiberhome.com)
 * @version 1.0
 * @date 2021-12-13
 * @copyright Copyright (c) 2021  烽火通信
 */
#ifndef __DRV_COMMON_H
#define __DRV_COMMON_H 

#include <rtthread.h>
#include <rthw.h>
#include <rtdevice.h>

void _Error_Handler(char *s, int num);

#ifndef Error_Handler
#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#endif


#endif	 // __DRV_COMMON_H 

/*********** (C) COPYRIGHT 2021 FiberHome *****END OF FILE****/
