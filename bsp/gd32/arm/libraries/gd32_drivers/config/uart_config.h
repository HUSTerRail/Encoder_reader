/**
 * @file drv_uart_config.h
 * @brief 串口配置
 * @author wangh (wanghuan3037@fiberhome.com)
 * @version 1.0
 * @date 2021-12-20
 * @copyright Copyright (c) 2021  烽火通信
 */
#ifndef __DRV_UART_CONFIG_H
#define __DRV_UART_CONFIG_H 
#include <rtthread.h>

#if defined(BSP_USING_UART0)
#ifndef UART0_CONFIG
#define UART0_CONFIG                    \
    {                                   \
        .name = "uart0",                \
        .uart_periph = USART0,          \
        .irqn = USART0_IRQn,            \
        .per_clk = RCU_USART0,          \
        .tx_gpio_clk = RCU_GPIOA,       \
        .rx_gpio_clk = RCU_GPIOA,       \
        .tx_port = GPIOA,               \
        .tx_pin = GPIO_PIN_9,           \
        .rx_port = GPIOA,               \
        .rx_pin = GPIO_PIN_10,          \
    }
#endif /* UART0_CONFIG */

#if defined(BSP_UART0_RX_USING_DMA)
#ifndef UART0_DMA_RX_CONFIG
#define UART0_DMA_RX_IRQHandler     DMA0_Channel4_IRQHandler
#define UART0_DMA_RX_CONFIG             \
    {                                   \
        .dma_periph = DMA0,             \
        .channelx = DMA_CH4,            \
        .dma_clk = RCU_DMA0,            \
        .dma_irq = DMA0_Channel4_IRQn,  \
    }
#endif /* UART0_DMA_RX_CONFIG */
#endif /* BSP_UART0_RX_USING_DMA */

#if defined(BSP_UART0_TX_USING_DMA)
#ifndef UART0_DMA_TX_CONFIG
#define UART0_DMA_TX_IRQHandler     DMA0_Channel3_IRQHandler
#define UART0_DMA_TX_CONFIG             \
    {                                   \
        .dma_periph = DMA0,             \
        .channelx = DMA_CH3,            \
        .dma_clk = RCU_DMA0,            \
        .dma_irq = DMA0_Channel3_IRQn,  \
    }
#endif /* UART0_DMA_TX_CONFIG */
#endif /* BSP_UART0_TX_USING_DMA */
#endif /* BSP_USING_UART0 */


#if defined(BSP_USING_UART1)
#ifndef UART1_CONFIG
#define UART1_CONFIG                    \
    {                                   \
        .name = "uart1",                \
        .uart_periph = USART1,          \
        .irqn = USART1_IRQn,            \
        .per_clk = RCU_USART1,          \
        .tx_gpio_clk = RCU_GPIOA,       \
        .rx_gpio_clk = RCU_GPIOA,       \
        .tx_port = GPIOA,               \
        .tx_pin = GPIO_PIN_2,           \
        .rx_port = GPIOA,               \
        .rx_pin = GPIO_PIN_3,          \
    }
#endif /* UART1_CONFIG */

#if defined(BSP_UART1_RX_USING_DMA)
#ifndef UART1_DMA_RX_CONFIG
#define UART1_DMA_RX_IRQHandler     DMA0_Channel5_IRQHandler
#define UART1_DMA_RX_CONFIG             \
    {                                   \
        .dma_periph = DMA0,             \
        .channelx = DMA_CH5,            \
        .dma_clk = RCU_DMA0,            \
        .dma_irq = DMA0_Channel5_IRQn,  \
    }
#endif /* UART1_DMA_RX_CONFIG */
#endif /* BSP_UART1_RX_USING_DMA */

#if defined(BSP_UART1_TX_USING_DMA)
#ifndef UART1_DMA_TX_CONFIG
#define UART1_DMA_TX_IRQHandler     DMA0_Channel6_IRQHandler
#define UART1_DMA_TX_CONFIG             \
    {                                   \
        .dma_periph = DMA0,             \
        .channelx = DMA_CH6,            \
        .dma_clk = RCU_DMA0,            \
        .dma_irq = DMA0_Channel6_IRQn,  \
    }
#endif /* UART1_DMA_TX_CONFIG */
#endif /* BSP_UART1_TX_USING_DMA */
#endif /* BSP_USING_UART1 */


#if defined(BSP_USING_UART2)
#ifndef UART2_CONFIG
#define UART2_CONFIG                    \
    {                                   \
        .name = "uart2",                \
        .uart_periph = USART2,          \
        .irqn = USART2_IRQn,            \
        .per_clk = RCU_USART2,          \
        .tx_gpio_clk = RCU_GPIOB,       \
        .rx_gpio_clk = RCU_GPIOB,       \
        .tx_port = GPIOB,               \
        .tx_pin = GPIO_PIN_10,           \
        .rx_port = GPIOB,               \
        .rx_pin = GPIO_PIN_11,          \
    }
#endif /* UART2_CONFIG */

#if defined(BSP_UART2_RX_USING_DMA)
#ifndef UART2_DMA_RX_CONFIG
#define UART2_DMA_RX_IRQHandler     DMA0_Channel2_IRQHandler
#define UART2_DMA_RX_CONFIG             \
    {                                   \
        .dma_periph = DMA0,             \
        .channelx = DMA_CH2,            \
        .dma_clk = RCU_DMA0,            \
        .dma_irq = DMA0_Channel2_IRQn,  \
    }
#endif /* UART2_DMA_RX_CONFIG */
#endif /* BSP_UART2_RX_USING_DMA */

#if defined(BSP_UART2_TX_USING_DMA)
#ifndef UART2_DMA_TX_CONFIG
#define UART2_DMA_TX_IRQHandler     DMA0_Channel1_IRQHandler
#define UART2_DMA_TX_CONFIG             \
    {                                   \
        .dma_periph = DMA0,             \
        .channelx = DMA_CH1,            \
        .dma_clk = RCU_DMA0,            \
        .dma_irq = DMA0_Channel1_IRQn,  \
    }
#endif /* UART2_DMA_TX_CONFIG */
#endif /* BSP_UART2_TX_USING_DMA */
#endif /* BSP_USING_UART2 */


#if defined(BSP_USING_UART3)
#ifndef UART3_CONFIG
#define UART3_CONFIG                    \
    {                                   \
        .name = "uart3",                \
        .uart_periph = UART3,           \
        .irqn = UART3_IRQn,             \
        .per_clk = RCU_UART3,           \
        .tx_gpio_clk = RCU_GPIOC,       \
        .rx_gpio_clk = RCU_GPIOC,       \
        .tx_port = GPIOC,               \
        .tx_pin = GPIO_PIN_10,           \
        .rx_port = GPIOC,               \
        .rx_pin = GPIO_PIN_11,          \
    }
#endif /* UART3_CONFIG */

#if defined(BSP_UART3_RX_USING_DMA)
#ifndef UART3_DMA_RX_CONFIG
#define UART3_DMA_RX_IRQHandler     DMA1_Channel2_IRQHandler
#define UART3_DMA_RX_CONFIG             \
    {                                   \
        .dma_periph = DMA1,             \
        .channelx = DMA_CH2,            \
        .dma_clk = RCU_DMA1,            \
        .dma_irq = DMA1_Channel2_IRQn,  \
    }
#endif /* UART3_DMA_RX_CONFIG */
#endif /* BSP_UART3_RX_USING_DMA */

#if defined(BSP_UART3_TX_USING_DMA)
#ifndef UART3_DMA_TX_CONFIG
#define UART3_DMA_TX_IRQHandler     DMA1_Channel4_IRQHandler
#define UART3_DMA_TX_CONFIG             \
    {                                   \
        .dma_periph = DMA1,             \
        .channelx = DMA_CH4,            \
        .dma_clk = RCU_DMA1,            \
        .dma_irq = DMA1_Channel4_IRQn,  \
    }
#endif /* UART3_DMA_TX_CONFIG */
#endif /* BSP_UART3_TX_USING_DMA */
#endif /* BSP_USING_UART3 */


#if defined(BSP_USING_UART4)
#ifndef UART4_CONFIG
#define UART4_CONFIG                    \
    {                                   \
        .name = "uart4",                \
        .uart_periph = UART4,          \
        .irqn = UART4_IRQn,            \
        .per_clk = RCU_UART4,          \
        .tx_gpio_clk = RCU_GPIOC,       \
        .rx_gpio_clk = RCU_GPIOD,       \
        .tx_port = GPIOC,               \
        .tx_pin = GPIO_PIN_12,           \
        .rx_port = GPIOD,               \
        .rx_pin = GPIO_PIN_2,          \
    }
#endif /* UART4_CONFIG */

#if defined(BSP_UART4_RX_USING_DMA)
#ifndef UART4_DMA_RX_CONFIG
#define UART4_DMA_RX_IRQHandler     DMA1_Channel1_IRQHandler
#define UART4_DMA_RX_CONFIG             \
    {                                   \
        .dma_periph = DMA1,             \
        .channelx = DMA_CH1,            \
        .dma_clk = RCU_DMA1,            \
        .dma_irq = DMA1_Channel1_IRQn,  \
    }
#endif /* UART4_DMA_RX_CONFIG */
#endif /* BSP_UART4_RX_USING_DMA */

#if defined(BSP_UART4_TX_USING_DMA)
#ifndef UART4_DMA_TX_CONFIG
#define UART4_DMA_TX_IRQHandler     DMA1_Channel3_IRQHandler
#define UART4_DMA_TX_CONFIG             \
    {                                   \
        .dma_periph = DMA1,             \
        .channelx = DMA_CH3,            \
        .dma_clk = RCU_DMA1,            \
        .dma_irq = DMA1_Channel3_IRQn,  \
    }
#endif /* UART4_DMA_TX_CONFIG */
#endif /* BSP_UART4_TX_USING_DMA */
#endif /* BSP_USING_UART4 */


#if defined(BSP_USING_UART5)
#ifndef UART5_CONFIG
#define UART5_CONFIG                    \
    {                                   \
        .name = "uart5",                \
        .uart_periph = USART5,          \
        .irqn = USART5_IRQn,            \
        .per_clk = RCU_USART5,          \
        .tx_gpio_clk = RCU_GPIOC,       \
        .rx_gpio_clk = RCU_GPIOC,       \
        .tx_port = GPIOC,               \
        .tx_pin = GPIO_PIN_6,           \
        .rx_port = GPIOC,               \
        .rx_pin = GPIO_PIN_7,           \
    }
#endif /* UART5_CONFIG */

#if defined(BSP_UART5_RX_USING_DMA)
#ifndef UART5_DMA_RX_CONFIG
#define UART5_DMA_RX_IRQHandler     DMA1_Channel5_IRQHandler
#define UART5_DMA_RX_CONFIG             \
    {                                   \
        .dma_periph = DMA1,             \
        .channelx = DMA_CH5,            \
        .dma_clk = RCU_DMA1,            \
        .dma_irq = DMA1_Channel5_IRQn,  \
    }
#endif /* UART5_DMA_RX_CONFIG */
#endif /* BSP_UART5_RX_USING_DMA */

#if defined(BSP_UART5_TX_USING_DMA)
#ifndef UART5_DMA_TX_CONFIG
#define UART5_DMA_TX_IRQHandler     DMA1_Channel6_IRQHandler
#define UART5_DMA_TX_CONFIG             \
    {                                   \
        .dma_periph = DMA1,             \
        .channelx = DMA_CH6,            \
        .dma_clk = RCU_DMA1,            \
        .dma_irq = DMA1_Channel6_IRQn,  \
    }
#endif /* UART5_DMA_TX_CONFIG */
#endif /* BSP_UART5_TX_USING_DMA */
#endif /* BSP_USING_UART5 */


#if defined(BSP_USING_UART6)
#ifndef UART6_CONFIG
#define UART6_CONFIG                    \
    {                                   \
        .name = "uart6",                \
        .uart_periph = UART6,          \
        .irqn = UART6_IRQn,            \
        .per_clk = RCU_UART6,          \
        .tx_gpio_clk = RCU_GPIOE,       \
        .rx_gpio_clk = RCU_GPIOE,       \
        .tx_port = GPIOE,               \
        .tx_pin = GPIO_PIN_7,           \
        .rx_port = GPIOE,               \
        .rx_pin = GPIO_PIN_8,          \
    }
#endif /* UART6_CONFIG */

#if defined(BSP_UART6_RX_USING_DMA)
#ifndef UART6_DMA_RX_CONFIG
#define UART6_DMA_RX_IRQHandler     DMA1_Channel2_IRQHandler
#define UART6_DMA_RX_CONFIG             \
    {                                   \
        .dma_periph = DMA1,             \
        .channelx = DMA_CH2,            \
        .dma_clk = RCU_DMA1,            \
        .dma_irq = DMA1_Channel2_IRQn,  \
    }
#endif /* UART6_DMA_RX_CONFIG */
#endif /* BSP_UART6_RX_USING_DMA */

#if defined(BSP_UART6_TX_USING_DMA)
#ifndef UART6_DMA_TX_CONFIG
#define UART6_DMA_TX_IRQHandler     DMA1_Channel4_IRQHandler
#define UART6_DMA_TX_CONFIG             \
    {                                   \
        .dma_periph = DMA1,             \
        .channelx = DMA_CH4,            \
        .dma_clk = RCU_DMA1,            \
        .dma_irq = DMA1_Channel4_IRQn,  \
    }
#endif /* UART6_DMA_TX_CONFIG */
#endif /* BSP_UART6_TX_USING_DMA */
#endif /* BSP_USING_UART6 */


#if defined(BSP_USING_UART7)
#ifndef UART7_CONFIG
#define UART7_CONFIG                    \
    {                                   \
        .name = "uart7",                \
        .uart_periph = UART7,           \
        .irqn = UART7_IRQn,             \
        .per_clk = RCU_UART7,           \
        .tx_gpio_clk = RCU_GPIOE,       \
        .rx_gpio_clk = RCU_GPIOE,       \
        .tx_port = GPIOE,               \
        .tx_pin = GPIO_PIN_0,           \
        .rx_port = GPIOE,               \
        .rx_pin = GPIO_PIN_1,           \
    }
#endif /* UART7_CONFIG */

#if defined(BSP_UART7_RX_USING_DMA)
#ifndef UART7_DMA_RX_CONFIG
#define UART7_DMA_RX_IRQHandler     DMA1_Channel1_IRQHandler
#define UART7_DMA_RX_CONFIG             \
    {                                   \
        .dma_periph = DMA1,             \
        .channelx = DMA_CH1,            \
        .dma_clk = RCU_DMA1,            \
        .dma_irq = DMA1_Channel1_IRQn,  \
    }
#endif /* UART7_DMA_RX_CONFIG */
#endif /* BSP_UART7_RX_USING_DMA */

#if defined(BSP_UART7_TX_USING_DMA)
#ifndef UART7_DMA_TX_CONFIG
#define UART7_DMA_TX_IRQHandler     DMA1_Channel3_IRQHandler
#define UART7_DMA_TX_CONFIG             \
    {                                   \
        .dma_periph = DMA1,             \
        .channelx = DMA_CH3,            \
        .dma_clk = RCU_DMA1,            \
        .dma_irq = DMA1_Channel3_IRQn,  \
    }
#endif /* UART7_DMA_TX_CONFIG */
#endif /* BSP_UART7_TX_USING_DMA */
#endif /* BSP_USING_UART7 */



#endif  // __DRV_UART_CONFIG_H 

/*********** (C) COPYRIGHT 2021 FiberHome *****END OF FILE****/
