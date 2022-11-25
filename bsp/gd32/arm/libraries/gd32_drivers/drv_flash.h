/**
 * @file drv_flash.h
 * @brief 
 * @author wangh (wanghuan3037@fiberhome.com)
 * @version 1.0
 * @date 2021-12-20
 * @copyright Copyright (c) 2021  烽火通信
 */
#ifndef __DRV_FLASH_H
#define __DRV_FLASH_H 

#include <rtthread.h>
#include "rtdevice.h"
#include <rthw.h>
#include <drv_common.h>

#ifdef __cplusplus
extern "C" {
#endif

int gd32_flash_read(rt_uint32_t addr, rt_uint8_t *buf, size_t size);
int gd32_flash_write(rt_uint32_t addr, const rt_uint8_t *buf, size_t size);
int gd32_flash_erase(rt_uint32_t addr, size_t size);

#ifdef __cplusplus
}
#endif


#endif	 // __DRV_FLASH_H 

/*********** (C) COPYRIGHT 2021 FiberHome *****END OF FILE****/
