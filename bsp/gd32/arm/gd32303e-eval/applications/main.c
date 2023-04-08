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

/* defined the LED2 pin: PF0 */
#define LED_RUN GET_PIN(B, 4)  //runµÆ
#define LED_ERR GET_PIN(B, 3)  //ERRµÆ

int main(void)
{ 
    int count = 1;
//		rcu_periph_clock_enable(RCU_GPIOB);

//		gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_4);
//	  gpio_bit_reset(GPIOB, GPIO_PIN_4);
	
    /* set LED2 pin mode to output */ 
	  rt_pin_mode(LED_RUN, PIN_MODE_OUTPUT);
    rt_pin_mode(LED_ERR, PIN_MODE_OUTPUT); 
		rt_pin_write(LED_ERR, PIN_HIGH);
//		extern int uart_sample();
//		uart_sample();
    while (count++)
    {
				rt_pin_write(LED_RUN, PIN_HIGH);
//			  rt_pin_write(LED_ERR, PIN_LOW);
        rt_thread_mdelay(1000);
        rt_pin_write(LED_RUN, PIN_LOW);
        rt_thread_mdelay(1000);
    }

    return RT_EOK;
}
