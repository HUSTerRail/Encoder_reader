/**
 * @file pwm_config.h
 * @brief PWM引脚配置
 * @author wangh (wanghuan3037@fiberhome.com)
 * @version 1.0
 * @date 2022-03-24
 * @copyright Copyright (c) 2022  烽火通信
 */
#ifndef __PWM_CONFIG_H
#define __PWM_CONFIG_H

#include <rtthread.h>

#ifdef BSP_USING_PWM0
#ifndef PWM0_CONFIG
#define PWM0_CONFIG             \
    {                           \
        .timer_periph = TIMER0, \
        .name = "pwm0",         \
        .channel = 0            \
    }
#endif /* PWM1_CONFIG */
#endif /* BSP_USING_PWM1 */

#ifdef BSP_USING_PWM1
#ifndef PWM1_CONFIG
#define PWM1_CONFIG             \
    {                           \
        .timer_periph = TIMER1, \
        .name = "pwm1",         \
        .channel = 0            \
    }
#endif /* PWM1_CONFIG */
#endif /* BSP_USING_PWM1 */

#ifdef BSP_USING_PWM2
#ifndef PWM2_CONFIG
#define PWM2_CONFIG             \
    {                           \
        .timer_periph = TIMER2, \
        .name = "pwm2",         \
        .channel = 0            \
    }
#endif /* PWM2_CONFIG */
#endif /* BSP_USING_PWM2 */

#ifdef BSP_USING_PWM3
#ifndef PWM3_CONFIG
#define PWM3_CONFIG             \
    {                           \
        .timer_periph = TIMER3, \
        .name = "pwm3",         \
        .channel = 0            \
    }
#endif /* PWM3_CONFIG */
#endif /* BSP_USING_PWM3 */

#ifdef BSP_USING_PWM4
#ifndef PWM4_CONFIG
#define PWM4_CONFIG             \
    {                           \
        .timer_periph = TIMER4, \
        .name = "pwm4",         \
        .channel = 0            \
    }
#endif /* PWM4_CONFIG */
#endif /* BSP_USING_PWM4 */

#ifdef BSP_USING_PWM5
#ifndef PWM5_CONFIG
#define PWM5_CONFIG             \
    {                           \
        .timer_periph = TIMER5, \
        .name = "pwm5",         \
        .channel = 0            \
    }
#endif /* PWM5_CONFIG */
#endif /* BSP_USING_PWM5 */

#endif // __PWM_CONFIG_H

/*********** (C) COPYRIGHT 2022 FiberHome *****END OF FILE****/
