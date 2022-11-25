/**
 * @file drv_usart.c
 * @brief 适配 UART_V2
 * @author wangh (wanghuan3037@fiberhome.com)
 * @version 1.0
 * @date 2021-12-20
 * @copyright Copyright (c) 2021  烽火通信
 */ 
#include "drv_usart_v2.h"
#include <board.h>

#ifdef RT_USING_SERIAL_V2

#define DBG_TAG              "drv.usart"
#define DBG_LVL               DBG_LOG
#include <rtdbg.h>

#if !defined(BSP_USING_UART0) && !defined(BSP_USING_UART1) && \
    !defined(BSP_USING_UART2) && !defined(BSP_USING_UART3) && \
    !defined(BSP_USING_UART4) && !defined(BSP_USING_UART5) && \
    !defined(BSP_USING_UART6) && !defined(BSP_USING_UART7)
#error "Please define at least one UARTx"
#endif

#ifdef RT_SERIAL_USING_DMA
    static void _uart_dma_config(struct rt_serial_device *serial, rt_ubase_t flag);
#endif

enum
{
#ifdef BSP_USING_UART0
    UART0_INDEX,
#endif

#ifdef BSP_USING_UART1
    UART1_INDEX,
#endif

#ifdef BSP_USING_UART2
    UART2_INDEX,
#endif

#ifdef BSP_USING_UART3
    UART3_INDEX,
#endif

#ifdef BSP_USING_UART4
    UART4_INDEX,
#endif

#ifdef BSP_USING_UART5
    UART5_INDEX,
#endif

#ifdef BSP_USING_UART6
    UART6_INDEX,
#endif

#ifdef BSP_USING_UART7
    UART7_INDEX,
#endif
};

static struct gd32_uart_config uart_config[] =
{
#ifdef BSP_USING_UART0
    UART0_CONFIG,
#endif

#ifdef BSP_USING_UART1
    UART1_CONFIG,
#endif

#ifdef BSP_USING_UART2
    UART2_CONFIG,
#endif

#ifdef BSP_USING_UART3
    UART3_CONFIG,
#endif

#ifdef BSP_USING_UART4
    UART4_CONFIG,
#endif

#ifdef BSP_USING_UART5
    UART5_CONFIG,
#endif

#ifdef BSP_USING_UART6
    UART6_CONFIG,
#endif

#ifdef BSP_USING_UART7
    UART7_CONFIG,
#endif
};


static struct gd32_uart uart_obj[sizeof(uart_config) / sizeof(uart_config[0])] = {0};

static rt_err_t _uart_configure(struct rt_serial_device *serial, struct serial_configure *cfg)
{
    struct gd32_uart *uart;
    RT_ASSERT(serial != RT_NULL);
    RT_ASSERT(cfg != RT_NULL);

    uart = rt_container_of(serial, struct gd32_uart, serial);

    usart_disable(uart->config->uart_periph);
    usart_baudrate_set(uart->config->uart_periph, cfg->baud_rate);

    switch (cfg->data_bits)
    {
    case DATA_BITS_9:
        usart_word_length_set(uart->config->uart_periph, USART_WL_9BIT);
        break;

    default:
        usart_word_length_set(uart->config->uart_periph, USART_WL_8BIT);
        break;
    }

    switch (cfg->stop_bits)
    {
    case STOP_BITS_2:
        usart_stop_bit_set(uart->config->uart_periph, USART_STB_2BIT);
        break;
    default:
        usart_stop_bit_set(uart->config->uart_periph, USART_STB_1BIT);
        break;
    }

    switch (cfg->parity)
    {
    case PARITY_ODD:
        usart_parity_config(uart->config->uart_periph, USART_PM_ODD);
        break;
    case PARITY_EVEN:
        usart_parity_config(uart->config->uart_periph, USART_PM_EVEN);
        break;
    default:
        usart_parity_config(uart->config->uart_periph, USART_PM_NONE);
        break;
    }

#ifdef RT_SERIAL_USING_DMA
    uart->dma_rx_remaining_cnt = serial->config.rx_bufsz;
#endif

    usart_receive_config(uart->config->uart_periph, USART_RECEIVE_ENABLE);
    usart_transmit_config(uart->config->uart_periph, USART_TRANSMIT_ENABLE);
    usart_enable(uart->config->uart_periph);

    return RT_EOK;
}

static rt_err_t _uart_control(struct rt_serial_device *serial, int cmd, void *arg)
{
    struct gd32_uart *uart;

    rt_ubase_t ctrl_arg = (rt_ubase_t)arg;

    RT_ASSERT(serial != RT_NULL);
    uart = rt_container_of(serial, struct gd32_uart, serial);

    if(ctrl_arg & (RT_DEVICE_FLAG_RX_BLOCKING | RT_DEVICE_FLAG_RX_NON_BLOCKING))
    {
        if (uart->uart_dma_flag & RT_DEVICE_FLAG_DMA_RX)
            ctrl_arg = RT_DEVICE_FLAG_DMA_RX;
        else
            ctrl_arg = RT_DEVICE_FLAG_INT_RX;
    }
    else if(ctrl_arg & (RT_DEVICE_FLAG_TX_BLOCKING | RT_DEVICE_FLAG_TX_NON_BLOCKING))
    {
        if (uart->uart_dma_flag & RT_DEVICE_FLAG_DMA_TX)
            ctrl_arg = RT_DEVICE_FLAG_DMA_TX;
        else
            ctrl_arg = RT_DEVICE_FLAG_INT_TX;
    }

    switch (cmd)
    {
    case RT_DEVICE_CTRL_CLR_INT:
        /* disable rx irq */
        NVIC_DisableIRQ(uart->config->irqn);
        /* disable interrupt */
        usart_interrupt_disable(uart->config->uart_periph, USART_INT_RBNE);

        NVIC_DisableIRQ(uart->config->irqn);
        if (ctrl_arg == RT_DEVICE_FLAG_INT_RX)
            usart_interrupt_disable(uart->config->uart_periph, USART_INT_RBNE);
        else if (ctrl_arg == RT_DEVICE_FLAG_INT_TX)
            usart_interrupt_disable(uart->config->uart_periph, USART_INT_TBE);
#ifdef RT_SERIAL_USING_DMA
        else if (ctrl_arg == RT_DEVICE_FLAG_DMA_RX)
        {
            usart_interrupt_disable(uart->config->uart_periph, USART_INT_RBNE);
            NVIC_DisableIRQ(uart->config->dma_rx->dma_irq);
            dma_deinit(uart->config->dma_rx->dma_periph, (dma_channel_enum)uart->config->dma_rx->channelx);
        }
        else if(ctrl_arg == RT_DEVICE_FLAG_DMA_TX)
        {
            usart_interrupt_disable(uart->config->uart_periph, USART_INT_TC);
            NVIC_DisableIRQ(uart->config->dma_tx->dma_irq);
            dma_deinit(uart->config->dma_tx->dma_periph, (dma_channel_enum)uart->config->dma_tx->channelx);
        }
#endif
        break;

    case RT_DEVICE_CTRL_SET_INT:
        nvic_irq_enable(uart->config->irqn, 3, 0);
        usart_interrupt_enable(uart->config->uart_periph, USART_INT_ERR);

        if (ctrl_arg == RT_DEVICE_FLAG_INT_RX)
        {
            usart_interrupt_enable(uart->config->uart_periph, USART_INT_RBNE);
            // usart_interrupt_enable(uart->config->uart_periph, USART_INT_IDLE);
        }
        else if (ctrl_arg == RT_DEVICE_FLAG_INT_TX)
            usart_interrupt_enable(uart->config->uart_periph, USART_INT_TBE);
        break;

    case RT_DEVICE_CTRL_CONFIG:
        if (ctrl_arg & (RT_DEVICE_FLAG_DMA_RX | RT_DEVICE_FLAG_DMA_TX))
        {

#ifdef RT_SERIAL_USING_DMA
            _uart_dma_config(serial, ctrl_arg);
#endif
        }
        else
            _uart_control(serial, RT_DEVICE_CTRL_SET_INT, (void *)ctrl_arg);
        break;

    case RT_DEVICE_CHECK_OPTMODE:
        {
            if (ctrl_arg & RT_DEVICE_FLAG_DMA_TX)
                return RT_SERIAL_TX_BLOCKING_NO_BUFFER;
            else
                return RT_SERIAL_TX_BLOCKING_BUFFER;
        }
    case RT_DEVICE_CTRL_CLOSE:
        usart_deinit(uart->config->uart_periph);
        break;
    }
    return RT_EOK;
}

static int _uart_putc(struct rt_serial_device *serial, char ch)
{
    struct gd32_uart *uart;

    RT_ASSERT(serial != RT_NULL);
    uart = rt_container_of(serial, struct gd32_uart, serial);

    usart_data_transmit(uart->config->uart_periph, ch);
    while((usart_flag_get(uart->config->uart_periph, USART_FLAG_TC) == RESET));

    return RT_EOK;
}

static int _uart_getc(struct rt_serial_device *serial)
{
    int ch;
    struct gd32_uart *uart;

    RT_ASSERT(serial != RT_NULL);
    uart = rt_container_of(serial, struct gd32_uart, serial);

    ch = -1;
    if (usart_flag_get(uart->config->uart_periph, USART_FLAG_RBNE) != RESET)
        ch = usart_data_receive(uart->config->uart_periph);
    return ch;
}

static rt_size_t _uart_transmit(struct rt_serial_device     *serial,
                                       rt_uint8_t           *buf,
                                       rt_size_t             size,
                                       rt_uint32_t           tx_flag)
{
    struct gd32_uart *uart;

    RT_ASSERT(serial != RT_NULL);
    RT_ASSERT(buf != RT_NULL);
    uart = rt_container_of(serial, struct gd32_uart, serial);

    if (uart->uart_dma_flag & RT_DEVICE_FLAG_DMA_TX)
    {
        /* TODO:UART DMA发送 */
        // HAL_UART_Transmit_DMA(&uart->handle, buf, size);
        return size;
    }
    // LOG_D("RT_DEVICE_CTRL_SET_INT\r\n");
    _uart_control(serial, RT_DEVICE_CTRL_SET_INT, (void *)tx_flag);

    return size;
}

#ifdef RT_SERIAL_USING_DMA
static void dma_recv_isr(struct rt_serial_device *serial, rt_uint8_t isr_flag)
{
    struct gd32_uart *uart;
    rt_base_t level;
    rt_size_t recv_len, counter;

    RT_ASSERT(serial != RT_NULL);
    uart = rt_container_of(serial, struct gd32_uart, serial);

    level = rt_hw_interrupt_disable();
    recv_len = 0;
    counter = dma_transfer_number_get( uart->config->dma_rx->dma_periph, (dma_channel_enum)uart->config->dma_rx->channelx );

    switch (isr_flag)
    {
    case UART_RX_DMA_IT_IDLE_FLAG:
        if (counter <= uart->dma_rx_remaining_cnt)
            recv_len = uart->dma_rx_remaining_cnt - counter;
        else
            recv_len = serial->config.rx_bufsz + uart->dma_rx_remaining_cnt - counter;
        break;

    case UART_RX_DMA_IT_HT_FLAG:
        if (counter < uart->dma_rx_remaining_cnt)
            recv_len = uart->dma_rx_remaining_cnt - counter;
        break;

    case UART_RX_DMA_IT_TC_FLAG:
        if(counter >= uart->dma_rx_remaining_cnt)
            recv_len = serial->config.rx_bufsz + uart->dma_rx_remaining_cnt - counter;

    default:
        break;
    }

    if (recv_len)
    {
        uart->dma_rx_remaining_cnt = counter;
        rt_hw_serial_isr(serial, RT_SERIAL_EVENT_RX_DMADONE | (recv_len << 8));
    }
    rt_hw_interrupt_enable(level);

}
#endif  /* RT_SERIAL_USING_DMA */


/**
 * Uart common interrupt process. This need add to uart ISR.
 *
 * @param serial serial device
 */
static void uart_isr(struct rt_serial_device *serial)
{
    struct gd32_uart *uart;

    RT_ASSERT(serial != RT_NULL);
    uart = rt_container_of(serial, struct gd32_uart, serial);

    /* If the Read data register is not empty and the RXNE interrupt is enabled  （RDR） */
    if ((usart_interrupt_flag_get(uart->config->uart_periph, USART_INT_FLAG_RBNE) != RESET) &&
             (usart_flag_get(uart->config->uart_periph, USART_FLAG_RBNE) != RESET))
    {
        struct rt_serial_rx_fifo *rx_fifo;
        rx_fifo = (struct rt_serial_rx_fifo *) serial->serial_rx;
        RT_ASSERT(rx_fifo != RT_NULL);

        rt_ringbuffer_putchar(&(rx_fifo->rb), (uint8_t)usart_data_receive(uart->config->uart_periph));

        rt_hw_serial_isr(serial, RT_SERIAL_EVENT_RX_IND);

        /* Clear RXNE interrupt flag */
        usart_flag_clear(uart->config->uart_periph, USART_FLAG_RBNE);
        usart_interrupt_flag_clear(uart->config->uart_periph, USART_INT_FLAG_RBNE);
    }

    /* If the Transmit data register is empty and the TXE interrupt enable is enabled  （TDR）*/
    else if ((usart_interrupt_flag_get(uart->config->uart_periph, USART_INT_FLAG_TBE) != RESET) &&
             (usart_flag_get(uart->config->uart_periph, USART_FLAG_TBE) != RESET))
    {
        struct rt_serial_tx_fifo *tx_fifo;
        tx_fifo = (struct rt_serial_tx_fifo *) serial->serial_tx;
        RT_ASSERT(tx_fifo != RT_NULL);

        rt_uint8_t put_char = 0;
        if (rt_ringbuffer_getchar(&(tx_fifo->rb), &put_char))
        {
            usart_data_transmit(uart->config->uart_periph, put_char);
        }
        else
        {
            usart_interrupt_disable(uart->config->uart_periph, USART_INT_TBE);
            usart_interrupt_enable(uart->config->uart_periph, USART_INT_TC);
        }
    }
    else if ((usart_interrupt_flag_get(uart->config->uart_periph, USART_INT_FLAG_TC) != RESET) &&
             (usart_flag_get(uart->config->uart_periph, USART_FLAG_TC) != RESET))
    {
        if (uart->uart_dma_flag & RT_DEVICE_FLAG_DMA_TX)
        {
            /* TODO:The HAL_UART_TxCpltCallback will be triggered */
            // HAL_UART_IRQHandler(&(uart->handle));
        }
        else
        {
            /* Transmission complete interrupt disable ( CR1 Register) */
            usart_interrupt_disable(uart->config->uart_periph, USART_INT_TC);
            rt_hw_serial_isr(serial, RT_SERIAL_EVENT_TX_DONE);
        }
        /* Clear Transmission complete interrupt flag ( ISR Register ) */
        usart_flag_clear(uart->config->uart_periph, USART_FLAG_TC);
        usart_interrupt_flag_clear(uart->config->uart_periph, USART_INT_FLAG_TC);
    }

    else if ((uart->uart_dma_flag) && (usart_flag_get(uart->config->uart_periph, USART_FLAG_IDLE) != RESET)
             && (usart_interrupt_flag_get(uart->config->uart_periph, USART_INT_FLAG_IDLE) != RESET))
    {
        if (uart->uart_dma_flag & RT_DEVICE_FLAG_DMA_RX)
            dma_recv_isr(serial, UART_RX_DMA_IT_IDLE_FLAG);
        else
            rt_hw_serial_isr(serial, RT_SERIAL_EVENT_RX_IND);   /* 中断接收，空闲通知 */

        usart_data_receive(uart->config->uart_periph);  /* Clear IDLE interrupt flag bit */
    }

    else
    {
        if (usart_flag_get(uart->config->uart_periph, USART_FLAG_ORERR) != RESET ||
            usart_flag_get(uart->config->uart_periph, USART_FLAG_NERR) != RESET ||
            usart_flag_get(uart->config->uart_periph, USART_FLAG_FERR) != RESET ||
            usart_flag_get(uart->config->uart_periph, USART_FLAG_PERR) != RESET)
        {
            LOG_E("(%s) serial device error!\r\n", serial->parent.parent.name);
            usart_data_receive(uart->config->uart_periph);  /* Clear interrupt flag bit */
        }
        if (usart_flag_get(uart->config->uart_periph, USART_FLAG_TC) != RESET)
        {
            usart_flag_clear(uart->config->uart_periph, USART_FLAG_TC);
        }
        if (usart_flag_get(uart->config->uart_periph, USART_FLAG_RBNE) != RESET)
        {
            usart_flag_clear(uart->config->uart_periph, USART_FLAG_RBNE);
        }
    }
}

#ifdef RT_SERIAL_USING_DMA
/**
 * Uart common interrupt process. This need add to uart ISR.
 *
 * @param serial serial device
 */
void uart_dma_isr(struct rt_serial_device *serial)
{
    struct gd32_uart *uart;

    RT_ASSERT(serial != RT_NULL);
    uart = rt_container_of(serial, struct gd32_uart, serial);

    if ((dma_flag_get(uart->config->dma_rx->dma_periph, (dma_channel_enum)uart->config->dma_rx->channelx, DMA_FLAG_HTF) != RESET) &&
        (dma_interrupt_flag_get(uart->config->dma_rx->dma_periph, (dma_channel_enum)uart->config->dma_rx->channelx, DMA_INT_FLAG_HTF) != RESET))
    {
        dma_recv_isr(&(uart->serial), UART_RX_DMA_IT_HT_FLAG);
        dma_flag_clear(uart->config->dma_rx->dma_periph, (dma_channel_enum)uart->config->dma_rx->channelx, DMA_FLAG_HTF);
        dma_interrupt_flag_clear(uart->config->dma_rx->dma_periph, (dma_channel_enum)uart->config->dma_rx->channelx, DMA_INT_FLAG_HTF);
    }
    else if ((dma_flag_get(uart->config->dma_rx->dma_periph, (dma_channel_enum)uart->config->dma_rx->channelx, DMA_FLAG_FTF) != RESET) &&
        (dma_interrupt_flag_get(uart->config->dma_rx->dma_periph, (dma_channel_enum)uart->config->dma_rx->channelx, DMA_INT_FLAG_FTF) != RESET))
    {
        dma_recv_isr(&(uart->serial), UART_RX_DMA_IT_TC_FLAG);
        dma_flag_clear(uart->config->dma_rx->dma_periph, (dma_channel_enum)uart->config->dma_rx->channelx, DMA_FLAG_FTF);
        dma_interrupt_flag_clear(uart->config->dma_rx->dma_periph, (dma_channel_enum)uart->config->dma_rx->channelx, DMA_INT_FLAG_FTF);
    }
    else
    {
        if (dma_flag_get(uart->config->dma_rx->dma_periph, (dma_channel_enum)uart->config->dma_rx->channelx, DMA_FLAG_HTF) != RESET)
        {
            dma_flag_clear(uart->config->dma_rx->dma_periph, (dma_channel_enum)uart->config->dma_rx->channelx, DMA_FLAG_HTF);
        }
        if (dma_flag_get(uart->config->dma_rx->dma_periph, (dma_channel_enum)uart->config->dma_rx->channelx, DMA_FLAG_FTF) != RESET)
        {
            dma_flag_clear(uart->config->dma_rx->dma_periph, (dma_channel_enum)uart->config->dma_rx->channelx, DMA_FLAG_FTF);
        }
        if (dma_flag_get(uart->config->dma_rx->dma_periph, (dma_channel_enum)uart->config->dma_rx->channelx, DMA_FLAG_ERR) != RESET)
        {
            dma_flag_clear(uart->config->dma_rx->dma_periph, (dma_channel_enum)uart->config->dma_rx->channelx, DMA_FLAG_ERR);
        }
    }
}

static void _uart_dma_config(struct rt_serial_device *serial, rt_ubase_t flag)
{
    struct rt_serial_rx_fifo *rx_fifo;
    struct dma_config *dma_config;
    struct gd32_uart *uart;

    RT_ASSERT(serial != RT_NULL);
    uart = rt_container_of(serial, struct gd32_uart, serial);

    if (RT_DEVICE_FLAG_DMA_RX == flag)
    {
        dma_config = uart->config->dma_rx;
    }
    else if (RT_DEVICE_FLAG_DMA_TX == flag)
    {
        dma_config = uart->config->dma_tx;
    }
    LOG_D("%s dma config start\r\n", uart->config->name);

    rcu_periph_clock_enable((rcu_periph_enum)dma_config->dma_clk);

    dma_parameter_struct dma_init_struct;
    dma_deinit(dma_config->dma_periph, (dma_channel_enum)dma_config->channelx);

    dma_init_struct.periph_addr = (uint32_t)&USART_DATA(uart->config->uart_periph);
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
    dma_init_struct.periph_inc = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_init_struct.memory_inc = DMA_MEMORY_INCREASE_ENABLE;
    dma_init_struct.priority = DMA_PRIORITY_ULTRA_HIGH;

    if (flag == RT_DEVICE_FLAG_DMA_RX)
    {
        dma_init_struct.direction = DMA_PERIPHERAL_TO_MEMORY;
        dma_circulation_enable(dma_config->dma_periph, (dma_channel_enum)dma_config->channelx);

        rx_fifo = (struct rt_serial_rx_fifo *)serial->serial_rx;
        RT_ASSERT(rx_fifo != RT_NULL);
        dma_init_struct.memory_addr = (uint32_t)rx_fifo->buffer;
        dma_init_struct.number = serial->config.rx_bufsz;

        usart_dma_receive_config(uart->config->uart_periph, USART_DENR_ENABLE);
        usart_interrupt_enable(uart->config->uart_periph, USART_INT_IDLE);
        usart_interrupt_enable(uart->config->uart_periph, USART_INT_ERR);
    }
    else if (flag == RT_DEVICE_FLAG_DMA_TX)
    {
        dma_init_struct.direction = DMA_MEMORY_TO_PERIPHERAL;
        dma_circulation_disable(dma_config->dma_periph, (dma_channel_enum)dma_config->channelx);
        usart_dma_receive_config(uart->config->uart_periph, USART_DENT_ENABLE);
    }

    dma_init(dma_config->dma_periph, (dma_channel_enum)dma_config->channelx, &dma_init_struct);
    dma_memory_to_memory_disable(dma_config->dma_periph, (dma_channel_enum)dma_config->channelx);

    dma_flag_clear(dma_config->dma_periph, (dma_channel_enum)dma_config->channelx, DMA_FLAG_G);
    dma_channel_enable(dma_config->dma_periph, (dma_channel_enum)dma_config->channelx);


    dma_interrupt_enable(dma_config->dma_periph, (dma_channel_enum)dma_config->channelx, DMA_INT_FLAG_FTF);
    dma_interrupt_enable(dma_config->dma_periph, (dma_channel_enum)dma_config->channelx, DMA_INT_FLAG_HTF);
    dma_interrupt_enable(dma_config->dma_periph, (dma_channel_enum)dma_config->channelx, DMA_INT_FLAG_ERR);

    nvic_irq_enable(dma_config->dma_irq, 3, 0);
    nvic_irq_enable(uart->config->irqn, 3, 0);
}
#endif  /* RT_SERIAL_USING_DMA */


#if defined(BSP_USING_UART0)
void USART0_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    uart_isr(&(uart_obj[UART0_INDEX].serial));

    /* leave interrupt */
    rt_interrupt_leave();
}
#if defined(RT_SERIAL_USING_DMA) && defined(BSP_UART0_RX_USING_DMA)
void UART0_DMA_RX_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    uart_dma_isr(&(uart_obj[UART0_INDEX].serial));

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* defined(RT_SERIAL_USING_DMA) && defined(BSP_UART0_RX_USING_DMA) */
#if defined(RT_SERIAL_USING_DMA) && defined(BSP_UART0_TX_USING_DMA)
void UART0_DMA_TX_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    /* 暂不使用 */

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* defined(RT_SERIAL_USING_DMA) && defined(BSP_UART0_TX_USING_DMA) */
#endif /* BSP_USING_UART0 */

#if defined(BSP_USING_UART1)
void USART1_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    uart_isr(&(uart_obj[UART1_INDEX].serial));

    /* leave interrupt */
    rt_interrupt_leave();
}
#if defined(RT_SERIAL_USING_DMA) && defined(BSP_UART1_RX_USING_DMA)
void UART1_DMA_RX_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    uart_dma_isr(&(uart_obj[UART1_INDEX].serial));

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* defined(RT_SERIAL_USING_DMA) && defined(BSP_UART1_RX_USING_DMA) */
#if defined(RT_SERIAL_USING_DMA) && defined(BSP_UART1_TX_USING_DMA)
void UART1_DMA_TX_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    /* 暂不使用 */

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* defined(RT_SERIAL_USING_DMA) && defined(BSP_UART1_TX_USING_DMA) */
#endif /* BSP_USING_UART1 */

#if defined(BSP_USING_UART2)
void USART2_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    uart_isr(&(uart_obj[UART2_INDEX].serial));

    /* leave interrupt */
    rt_interrupt_leave();
}
#if defined(RT_SERIAL_USING_DMA) && defined(BSP_UART2_RX_USING_DMA)
void UART2_DMA_RX_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    uart_dma_isr(&(uart_obj[UART2_INDEX].serial));

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* defined(RT_SERIAL_USING_DMA) && defined(BSP_UART2_RX_USING_DMA) */
#if defined(RT_SERIAL_USING_DMA) && defined(BSP_UART2_TX_USING_DMA)
void UART2_DMA_TX_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    /* 暂不使用 */

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* defined(RT_SERIAL_USING_DMA) && defined(BSP_UART2_TX_USING_DMA) */
#endif /* BSP_USING_UART2 */

#if defined(BSP_USING_UART3)
void UART3_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    uart_isr(&(uart_obj[UART3_INDEX].serial));

    /* leave interrupt */
    rt_interrupt_leave();
}
#if defined(RT_SERIAL_USING_DMA) && defined(BSP_UART3_RX_USING_DMA)
void UART3_DMA_RX_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    uart_dma_isr(&(uart_obj[UART3_INDEX].serial));

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* defined(BSP_UART_USING_DMA_RX) && defined(BSP_UART3_RX_USING_DMA) */
#if defined(RT_SERIAL_USING_DMA) && defined(BSP_UART3_TX_USING_DMA)
void UART3_DMA_TX_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    /* 暂不使用 */

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* defined(BSP_UART_USING_DMA_TX) && defined(BSP_UART3_TX_USING_DMA) */
#endif /* BSP_USING_UART3 */

#if defined(BSP_USING_UART4)
void UART4_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    uart_isr(&(uart_obj[UART4_INDEX].serial));

    /* leave interrupt */
    rt_interrupt_leave();
}
#if defined(RT_SERIAL_USING_DMA) && defined(BSP_UART4_RX_USING_DMA)
void UART4_DMA_RX_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    uart_dma_isr(&(uart_obj[UART4_INDEX].serial));

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* defined(BSP_UART_USING_DMA_RX) && defined(BSP_UART4_RX_USING_DMA) */

#if defined(RT_SERIAL_USING_DMA) && defined(BSP_UART4_TX_USING_DMA)
void UART4_DMA_TX_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    /* 暂不使用 */

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* defined(BSP_UART_USING_DMA_TX) && defined(BSP_UART4_TX_USING_DMA) */
#endif /* BSP_USING_UART4 */

#if defined(BSP_USING_UART5)
void USART5_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    uart_isr(&(uart_obj[UART5_INDEX].serial));

    /* leave interrupt */
    rt_interrupt_leave();
}
#if defined(RT_SERIAL_USING_DMA) && defined(BSP_UART5_RX_USING_DMA)
void UART5_DMA_RX_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    uart_dma_isr(&(uart_obj[UART5_INDEX].serial));

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* defined(RT_SERIAL_USING_DMA) && defined(BSP_UART5_RX_USING_DMA) */
#if defined(RT_SERIAL_USING_DMA) && defined(BSP_UART5_TX_USING_DMA)
void UART5_DMA_TX_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    /* 暂不使用 */

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* defined(RT_SERIAL_USING_DMA) && defined(BSP_UART5_TX_USING_DMA) */
#endif /* BSP_USING_UART5 */

#if defined(BSP_USING_UART6)
void UART6_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    uart_isr(&(uart_obj[UART6_INDEX].serial));

    /* leave interrupt */
    rt_interrupt_leave();
}
#if defined(RT_SERIAL_USING_DMA) && defined(BSP_UART6_RX_USING_DMA)
void UART6_DMA_RX_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    uart_dma_isr(&(uart_obj[UART6_INDEX].serial));

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* defined(RT_SERIAL_USING_DMA) && defined(BSP_UART6_RX_USING_DMA) */
#if defined(RT_SERIAL_USING_DMA) && defined(BSP_UART6_TX_USING_DMA)
void UART6_DMA_TX_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    /* 暂不使用 */

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* defined(RT_SERIAL_USING_DMA) && defined(BSP_UART6_TX_USING_DMA) */
#endif /* BSP_USING_UART6 */

#if defined(BSP_USING_UART7)
void UART7_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    uart_isr(&(uart_obj[UART7_INDEX].serial));

    /* leave interrupt */
    rt_interrupt_leave();
}
#if defined(RT_SERIAL_USING_DMA) && defined(BSP_UART7_RX_USING_DMA)
void UART7_DMA_RX_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    uart_dma_isr(&(uart_obj[UART7_INDEX].serial));

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* defined(RT_SERIAL_USING_DMA) && defined(BSP_UART7_RX_USING_DMA) */
#if defined(RT_SERIAL_USING_DMA) && defined(BSP_UART7_TX_USING_DMA)
void UART7_DMA_TX_IRQHandler(void)
{
    /* enter interrupt */
    rt_interrupt_enter();

    /* 暂不使用 */

    /* leave interrupt */
    rt_interrupt_leave();
}
#endif /* defined(RT_SERIAL_USING_DMA) && defined(BSP_UART7_TX_USING_DMA) */
#endif /* BSP_USING_UART7 */


static void _uart_get_config(void)
{
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;

#ifdef BSP_USING_UART0
    uart_obj[UART0_INDEX].serial.config = config;
    uart_obj[UART0_INDEX].uart_dma_flag = 0;

    uart_obj[UART0_INDEX].serial.config.rx_bufsz = BSP_UART0_RX_BUFSIZE;
    uart_obj[UART0_INDEX].serial.config.tx_bufsz = BSP_UART0_TX_BUFSIZE;

#ifdef BSP_UART0_RX_USING_DMA
    uart_obj[UART0_INDEX].uart_dma_flag |= RT_DEVICE_FLAG_DMA_RX;
    static struct dma_config uart0_dma_rx = UART0_DMA_RX_CONFIG;
    uart_config[UART0_INDEX].dma_rx = &uart0_dma_rx;
#endif

#ifdef BSP_UART0_TX_USING_DMA
    uart_obj[UART0_INDEX].uart_dma_flag |= RT_DEVICE_FLAG_DMA_TX;
    static struct dma_config uart0_dma_tx = UART0_DMA_TX_CONFIG;
    uart_config[UART0_INDEX].dma_tx = &uart0_dma_tx;
#endif
#endif

#ifdef BSP_USING_UART1
    uart_obj[UART1_INDEX].serial.config = config;
    uart_obj[UART1_INDEX].uart_dma_flag = 0;

    uart_obj[UART1_INDEX].serial.config.rx_bufsz = BSP_UART1_RX_BUFSIZE;
    uart_obj[UART1_INDEX].serial.config.tx_bufsz = BSP_UART1_TX_BUFSIZE;

#ifdef BSP_UART1_RX_USING_DMA
    uart_obj[UART1_INDEX].uart_dma_flag |= RT_DEVICE_FLAG_DMA_RX;
    static struct dma_config uart1_dma_rx = UART1_DMA_RX_CONFIG;
    uart_config[UART1_INDEX].dma_rx = &uart1_dma_rx;
#endif

#ifdef BSP_UART1_TX_USING_DMA
    uart_obj[UART1_INDEX].uart_dma_flag |= RT_DEVICE_FLAG_DMA_TX;
    static struct dma_config uart1_dma_tx = UART1_DMA_TX_CONFIG;
    uart_config[UART1_INDEX].dma_tx = &uart1_dma_tx;
#endif
#endif

#ifdef BSP_USING_UART2
    uart_obj[UART2_INDEX].serial.config = config;
    uart_obj[UART2_INDEX].uart_dma_flag = 0;

    uart_obj[UART2_INDEX].serial.config.rx_bufsz = BSP_UART2_RX_BUFSIZE;
    uart_obj[UART2_INDEX].serial.config.tx_bufsz = BSP_UART2_TX_BUFSIZE;

#ifdef BSP_UART2_RX_USING_DMA
    uart_obj[UART2_INDEX].uart_dma_flag |= RT_DEVICE_FLAG_DMA_RX;
    static struct dma_config uart2_dma_rx = UART2_DMA_RX_CONFIG;
    uart_config[UART2_INDEX].dma_rx = &uart2_dma_rx;
#endif

#ifdef BSP_UART2_TX_USING_DMA
    uart_obj[UART2_INDEX].uart_dma_flag |= RT_DEVICE_FLAG_DMA_TX;
    static struct dma_config uart2_dma_tx = UART2_DMA_TX_CONFIG;
    uart_config[UART2_INDEX].dma_tx = &uart2_dma_tx;
#endif
#endif

#ifdef BSP_USING_UART3
    uart_obj[UART3_INDEX].serial.config = config;
    uart_obj[UART3_INDEX].uart_dma_flag = 0;

    uart_obj[UART3_INDEX].serial.config.rx_bufsz = BSP_UART3_RX_BUFSIZE;
    uart_obj[UART3_INDEX].serial.config.tx_bufsz = BSP_UART3_TX_BUFSIZE;

#ifdef BSP_UART3_RX_USING_DMA
    uart_obj[UART3_INDEX].uart_dma_flag |= RT_DEVICE_FLAG_DMA_RX;
    static struct dma_config uart3_dma_rx = UART3_DMA_RX_CONFIG;
    uart_config[UART3_INDEX].dma_rx = &uart3_dma_rx;
#endif

#ifdef BSP_UART3_TX_USING_DMA
    uart_obj[UART3_INDEX].uart_dma_flag |= RT_DEVICE_FLAG_DMA_TX;
    static struct dma_config uart3_dma_tx = UART3_DMA_TX_CONFIG;
    uart_config[UART3_INDEX].dma_tx = &uart3_dma_tx;
#endif
#endif

#ifdef BSP_USING_UART4
    uart_obj[UART4_INDEX].serial.config = config;
    uart_obj[UART4_INDEX].uart_dma_flag = 0;

    uart_obj[UART4_INDEX].serial.config.rx_bufsz = BSP_UART4_RX_BUFSIZE;
    uart_obj[UART4_INDEX].serial.config.tx_bufsz = BSP_UART4_TX_BUFSIZE;

#ifdef BSP_UART4_RX_USING_DMA
    uart_obj[UART4_INDEX].uart_dma_flag |= RT_DEVICE_FLAG_DMA_RX;
    static struct dma_config uart4_dma_rx = UART4_DMA_RX_CONFIG;
    uart_config[UART4_INDEX].dma_rx = &uart4_dma_rx;
#endif

#ifdef BSP_UART4_TX_USING_DMA
    uart_obj[UART4_INDEX].uart_dma_flag |= RT_DEVICE_FLAG_DMA_TX;
    static struct dma_config uart4_dma_tx = UART4_DMA_TX_CONFIG;
    uart_config[UART4_INDEX].dma_tx = &uart4_dma_tx;
#endif
#endif

#ifdef BSP_USING_UART5
    uart_obj[UART5_INDEX].serial.config = config;
    uart_obj[UART5_INDEX].uart_dma_flag = 0;

    uart_obj[UART5_INDEX].serial.config.rx_bufsz = BSP_UART5_RX_BUFSIZE;
    uart_obj[UART5_INDEX].serial.config.tx_bufsz = BSP_UART5_TX_BUFSIZE;

#ifdef BSP_UART5_RX_USING_DMA
    uart_obj[UART5_INDEX].uart_dma_flag |= RT_DEVICE_FLAG_DMA_RX;
    static struct dma_config uart5_dma_rx = UART5_DMA_RX_CONFIG;
    uart_config[UART5_INDEX].dma_rx = &uart5_dma_rx;
#endif

#ifdef BSP_UART5_TX_USING_DMA
    uart_obj[UART5_INDEX].uart_dma_flag |= RT_DEVICE_FLAG_DMA_TX;
    static struct dma_config uart5_dma_tx = UART5_DMA_TX_CONFIG;
    uart_config[UART5_INDEX].dma_tx = &uart5_dma_tx;
#endif
#endif

#ifdef BSP_USING_UART6
    uart_obj[UART6_INDEX].serial.config = config;
    uart_obj[UART6_INDEX].uart_dma_flag = 0;

    uart_obj[UART6_INDEX].serial.config.rx_bufsz = BSP_UART6_RX_BUFSIZE;
    uart_obj[UART6_INDEX].serial.config.tx_bufsz = BSP_UART6_TX_BUFSIZE;

#ifdef BSP_UART6_RX_USING_DMA
    uart_obj[UART6_INDEX].uart_dma_flag |= RT_DEVICE_FLAG_DMA_RX;
    static struct dma_config uart6_dma_rx = UART6_DMA_RX_CONFIG;
    uart_config[UART6_INDEX].dma_rx = &uart6_dma_rx;
#endif

#ifdef BSP_UART6_TX_USING_DMA
    uart_obj[UART6_INDEX].uart_dma_flag |= RT_DEVICE_FLAG_DMA_TX;
    static struct dma_config uart6_dma_tx = UART6_DMA_TX_CONFIG;
    uart_config[UART6_INDEX].dma_tx = &uart6_dma_tx;
#endif
#endif

#ifdef BSP_USING_UART7
    uart_obj[UART7_INDEX].serial.config = config;
    uart_obj[UART7_INDEX].uart_dma_flag = 0;

    uart_obj[UART7_INDEX].serial.config.rx_bufsz = BSP_UART7_RX_BUFSIZE;
    uart_obj[UART7_INDEX].serial.config.tx_bufsz = BSP_UART7_TX_BUFSIZE;

#ifdef BSP_UART7_RX_USING_DMA
    uart_obj[UART7_INDEX].uart_dma_flag |= RT_DEVICE_FLAG_DMA_RX;
    static struct dma_config uart7_dma_rx = UART7_DMA_RX_CONFIG;
    uart_config[UART7_INDEX].dma_rx = &uart7_dma_rx;
#endif

#ifdef BSP_UART7_TX_USING_DMA
    uart_obj[UART7_INDEX].uart_dma_flag |= RT_DEVICE_FLAG_DMA_TX;
    static struct dma_config uart7_dma_tx = UART7_DMA_TX_CONFIG;
    uart_config[UART7_INDEX].dma_tx = &uart7_dma_tx;
#endif
#endif
}


/**
* @brief UART MSP Initialization
*        This function configures the hardware resources used in this example:
*           - Peripheral's clock enable
*           - Peripheral's GPIO Configuration
*           - NVIC configuration for UART interrupt request enable
* @param uart: UART handle pointer
* @retval None
*/
static void gd32_uart_gpio_init(struct gd32_uart_config *pconfig)
{
    /* enable USART clock */
    rcu_periph_clock_enable(pconfig->tx_gpio_clk);
    rcu_periph_clock_enable(pconfig->rx_gpio_clk);
    rcu_periph_clock_enable(pconfig->per_clk);

    /* connect port to USARTx_Tx Rx */
    gpio_init(pconfig->tx_port, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, pconfig->tx_pin);
    gpio_init(pconfig->rx_port, GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, pconfig->rx_pin);

#ifdef RT_SERIAL_USING_DMA

#endif
}

static const struct rt_uart_ops gd32_uart_ops =
{
    .configure = _uart_configure,
    .control = _uart_control,
    .putc = _uart_putc,
    .getc = _uart_getc,
    .transmit = _uart_transmit
};

int rt_hw_usart_init(void)
{
    rt_err_t result = 0;
    rt_size_t obj_num = sizeof(uart_obj) / sizeof(struct gd32_uart);

    _uart_get_config();
    for (int i = 0; i < obj_num; i++)
    {
        /* init UART object */
        uart_obj[i].config = &uart_config[i];
        uart_obj[i].serial.ops = &gd32_uart_ops;
        gd32_uart_gpio_init(&uart_config[i]);

        /* register UART device */
        result = rt_hw_serial_register(&uart_obj[i].serial,
                                        uart_obj[i].config->name,
                                        RT_DEVICE_FLAG_RDWR,
                                        NULL);
        RT_ASSERT(result == RT_EOK);
    }

    return result;
}

#endif  /* RT_USING_SERIAL_V2 */


/*********** (C) COPYRIGHT 2021 FiberHome *****END OF FILE****/
