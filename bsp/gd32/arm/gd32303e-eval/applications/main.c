/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-08-20     BruceOu      first implementation
 */

#include <stdio.h>
#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>



#define SAMPLE_UART_NAME       "uart2"    /* 串口设备名称 */
static rt_device_t serial;                /* 串口设备句柄 */
char str[1] = {0X61};
struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT; /* 配置参数 */





/* defined the LED2 pin: PF0 */
#define LED_RUN GET_PIN(B, 4)  //run灯
#define LED_ERR GET_PIN(B, 3)  //ERR灯

extern int can_sample();

int main(void)
{ 
    int count = 1;

		rcu_periph_clock_enable(RCU_GPIOB);

		gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_4);
	  gpio_bit_reset(GPIOB, GPIO_PIN_4);
	
    /* set LED2 pin mode to output */ 
	  rt_pin_mode(LED_RUN, PIN_MODE_OUTPUT);
    rt_pin_mode(LED_ERR, PIN_MODE_OUTPUT); 
	
			/* 查找串口设备 */
		serial = rt_device_find(SAMPLE_UART_NAME);

		/* 以中断接收及轮询发送模式打开串口设备 */
		rt_device_open(serial, RT_DEVICE_FLAG_INT_RX);
	
    while (count++)
    {
        rt_pin_write(LED_RUN, PIN_HIGH);
        rt_thread_mdelay(500);
        rt_pin_write(LED_RUN, PIN_LOW);
        rt_thread_mdelay(500);
			  rt_pin_write(LED_ERR, PIN_HIGH);
        rt_thread_mdelay(500);
        rt_pin_write(LED_ERR, PIN_LOW);
        rt_thread_mdelay(500);
				rt_kprintf("a");
			/* 发送字符串 */
//				rt_device_write(serial, 0, str, 1);
    }

    return RT_EOK;
}