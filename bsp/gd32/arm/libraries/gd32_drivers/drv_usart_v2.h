/**
 * @file drv_usart_v2.h
 * @brief 
 * @author wangh (wanghuan3037@fiberhome.com)
 * @version 1.0
 * @date 2021-12-20
 * @copyright Copyright (c) 2021  烽火通信
 */
#ifndef __DRV_USART_V2_H
#define __DRV_USART_V2_H 

#include <rtthread.h>
#include <rtdevice.h>
#include <rthw.h>
#include <drv_dma.h>
#include "uart_config.h"

#define UART_RX_DMA_IT_IDLE_FLAG        0x00
#define UART_RX_DMA_IT_HT_FLAG          0x01
#define UART_RX_DMA_IT_TC_FLAG          0x02

/* gd32 config class */
struct gd32_uart_config
{
    const char *name;
    uint32_t uart_periph;           ///< UART外设
    IRQn_Type irqn;                 ///< UART中断编号
    rcu_periph_enum per_clk;        ///< UART外设时钟
    rcu_periph_enum tx_gpio_clk;    ///< UART-TX引脚gpio时钟
    rcu_periph_enum rx_gpio_clk;    ///< UART-RX引脚gpio时钟
    uint32_t tx_port;
    uint16_t tx_pin;
    uint32_t rx_port;
    uint16_t rx_pin;

#ifdef RT_SERIAL_USING_DMA
    struct dma_config *dma_rx;
    struct dma_config *dma_tx;
#endif
};


/* gd32 uart dirver class */
struct gd32_uart
{
    struct gd32_uart_config *config;

#ifdef RT_SERIAL_USING_DMA
    rt_size_t dma_rx_remaining_cnt;
#endif

    rt_uint16_t uart_dma_flag;
    struct rt_serial_device serial;
};

int rt_hw_usart_init(void);


#endif	 // __DRV_USART_V2_H 

/*********** (C) COPYRIGHT 2021 FiberHome *****END OF FILE****/
