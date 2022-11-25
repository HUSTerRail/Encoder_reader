/**
 * @file drv_common.c
 * @brief gd32适配rtthread驱动
 * @author wangh (wanghuan3037@fiberhome.com)
 * @version 1.0
 * @date 2021-12-13
 * @copyright Copyright (c) 2021  烽火通信
 */ 
#include "drv_common.h"
#include <board.h>

#ifdef RT_USING_FINSH
#include <finsh.h>
static void reboot(uint8_t argc, char **argv)
{
    rt_hw_cpu_reset();
}
FINSH_FUNCTION_EXPORT_ALIAS(reboot, __cmd_reboot, Reboot System);
#endif /* RT_USING_FINSH */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char *s, int num)
{
    /* USER CODE BEGIN Error_Handler */
    /* User can add his own implementation to report the HAL error return state */
    while (1)
    {
    }
    /* USER CODE END Error_Handler */
}

/** System Clock Configuration
*/
void rt_hw_systick_init(void)
{
    SysTick_Config(SystemCoreClock / RT_TICK_PER_SECOND);
    NVIC_SetPriority(SysTick_IRQn, 0);
}

/**
 * This is the timer interrupt service routine.
 *
 */
//void SysTick_Handler(void)
//{
//    /* enter interrupt */
//    rt_interrupt_enter();

//    rt_tick_increase();

//    /* leave interrupt */
//    rt_interrupt_leave();
//}

/**
 * This function will initial GD32 board.
 */
//void rt_hw_board_init()
//{
//    /* NVIC Configuration */
//#define NVIC_VTOR_MASK              0x3FFFFF80
//#ifdef  VECT_TAB_RAM
//    /* Set the Vector Table base location at 0x10000000 */
//    SCB->VTOR  = (0x10000000 & NVIC_VTOR_MASK);
//#else  /* VECT_TAB_FLASH  */
//    /* Set the Vector Table base location at 0x08000000 */
//    SCB->VTOR  = (0x08000000 & NVIC_VTOR_MASK);
//#endif

//#if 0
//    /* enable interrupt */
//    __set_PRIMASK(0);
//    /* System clock initialization */
//    SystemClock_Config();
//    /* disable interrupt */
//    __set_PRIMASK(1);
//#endif

//    rt_hw_systick_init();

//#ifdef BSP_USING_SDRAM
//    rt_system_heap_init((void *)EXT_SDRAM_BEGIN, (void *)EXT_SDRAM_END);
//#else
//    rt_system_heap_init((void *)HEAP_BEGIN, (void *)HEAP_END);
//#endif

//    rcu_periph_clock_enable(RCU_AF);    /* 使能复用时钟 */
//    nvic_priority_group_set(NVIC_PRIGROUP_PRE4_SUB0);

//    /* USART driver initialization is open by default */
//#ifdef RT_USING_SERIAL
//extern int rt_hw_usart_init(void);
//    rt_hw_usart_init();
//#endif

//    /* SEGGER_RTT debug support */
//#ifdef SEGGER_RTT_ENABLE
//extern int rt_hw_jlink_rtt_init(void);
//    rt_hw_jlink_rtt_init();
//#endif

//#ifdef RT_USING_CONSOLE
//    rt_console_set_device(RT_CONSOLE_DEVICE_NAME);
//#endif

//#ifdef RT_USING_COMPONENTS_INIT
//    rt_components_board_init();
//#endif
//}


///*********** (C) COPYRIGHT 2021 FiberHome *****END OF FILE****/
