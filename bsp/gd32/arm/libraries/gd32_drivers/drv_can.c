/**
 * @file drv_can.c
 * @brief GD32 CAN-RTT适配驱动 \n
 * @note 该版仅适配了GD32F10X
 * @author wangh (wanghuan3037@fiberhome.com)
 * @version 1.0
 * @date 2022-03-23
 * @copyright Copyright (c) 2022 
 */ 
#include "drv_can.h"

#ifdef RT_USING_CAN

#define DBG_TAG "drv.adc"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

struct gd32_baud_rate_tab
{
    uint32_t baud_rate;
    uint32_t bt_swj;
    uint32_t bt_bs2;
    uint32_t bt_bs1;
    uint32_t bt_psc;
};

/* baud = APB1 / (1 + BS1 + BS2) / PSC */
//#if defined (SOC_SERIES_GD32F1)     /* APB1 36MHz */
static const struct gd32_baud_rate_tab can_baud_rate_tab[] =
{
    {CAN1MBaud,   CAN_BT_SJW_2TQ, CAN_BT_BS1_8TQ, CAN_BT_BS2_3TQ, 3},
    {CAN800kBaud, CAN_BT_SJW_2TQ, CAN_BT_BS1_5TQ, CAN_BT_BS2_3TQ, 5},
    {CAN500kBaud, CAN_BT_SJW_2TQ, CAN_BT_BS1_8TQ, CAN_BT_BS2_3TQ, 6},
    {CAN250kBaud, CAN_BT_SJW_2TQ, CAN_BT_BS1_8TQ, CAN_BT_BS2_3TQ, 12},
    {CAN125kBaud, CAN_BT_SJW_2TQ, CAN_BT_BS1_8TQ, CAN_BT_BS2_3TQ, 24},
    {CAN100kBaud, CAN_BT_SJW_2TQ, CAN_BT_BS1_8TQ, CAN_BT_BS2_3TQ, 30},
    {CAN50kBaud,  CAN_BT_SJW_2TQ, CAN_BT_BS1_8TQ, CAN_BT_BS2_3TQ, 60},
    {CAN20kBaud,  CAN_BT_SJW_2TQ, CAN_BT_BS1_8TQ, CAN_BT_BS2_3TQ, 150},
    {CAN10kBaud,  CAN_BT_SJW_2TQ, CAN_BT_BS1_8TQ, CAN_BT_BS2_3TQ, 300},
};
//#elif defined (SOC_SERIES_GD32F2)   /* APB1 42MHz(max) */

//#endif


static gd32_can_config can_dev_config[] = 
{
#ifdef BSP_USING_CAN0
    CAN0_CONFIG,
#endif

#ifdef BSP_USING_CAN1
    CAN1_CONFIG,
#endif
};

static gd32_can_device can_devs[sizeof(can_dev_config) / sizeof(can_dev_config[0])] = {0};

/**
 * @brief 获取速率对应编号
 * @param[in]  baud     速率
 */
static rt_uint32_t get_can_baud_index(rt_uint32_t baud)
{
    rt_uint32_t len, index;

    len = sizeof(can_baud_rate_tab) / sizeof(can_baud_rate_tab[0]);
    for (index = 0; index < len; index++)
    {
        if (can_baud_rate_tab[index].baud_rate == baud)
            return index;
    }

    return 0;   /* default baud is CAN1MBaud */
}

/**
 * @brief can引脚初始化
 * @param[in]  config   外设配置
 */
RT_WEAK void gd32_can_gpio_init(gd32_can_config *config)
{
//    rcu_periph_clock_enable(config->per_clk);
	
		rcu_periph_clock_enable(RCU_CAN0);
		rcu_periph_clock_enable(RCU_GPIOA);
		gpio_init(GPIOA,GPIO_MODE_IPU,GPIO_OSPEED_50MHZ,GPIO_PIN_11);
    gpio_init(GPIOA,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_12);
	
//    rcu_periph_clock_enable((rcu_periph_enum)PIN_GDRCU(config->tx_pin));
//    rcu_periph_clock_enable((rcu_periph_enum)PIN_GDRCU(config->rx_pin));
//    gpio_init(PIN_GDPORT(config->tx_pin), GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, PIN_GDPIN(config->tx_pin));
//    gpio_init(PIN_GDPORT(config->rx_pin), GPIO_MODE_IPU, GPIO_OSPEED_50MHZ, PIN_GDPIN(config->rx_pin));

//    if (config->remap > 0)
//    {
        rcu_periph_clock_enable(RCU_AF);
				gpio_pin_remap_config(GPIO_CAN_FULL_REMAP,ENABLE);
//        gpio_pin_remap_config(config->remap, ENABLE);
//    }
}


static rt_err_t _can_config(struct rt_can_device *can, struct can_configure *cfg)
{
    RT_ASSERT(can != RT_NULL);
    RT_ASSERT(cfg != RT_NULL);

    gd32_can_device *can_dev = rt_container_of(can, gd32_can_device, device);
    rt_uint32_t baud_index = get_can_baud_index(cfg->baud_rate);

    can_parameter_struct            can_parameter;
    can_filter_parameter_struct     can_filter;

    can_struct_para_init(CAN_INIT_STRUCT, &can_parameter);
    can_struct_para_init(CAN_FILTER_STRUCT, &can_filter);
    
    can_deinit(can_dev->config->can_periph);
    can_parameter.time_triggered = DISABLE;
    can_parameter.auto_bus_off_recovery = DISABLE;
    can_parameter.auto_wake_up = DISABLE;
    can_parameter.no_auto_retrans = DISABLE;
    can_parameter.rec_fifo_overwrite = DISABLE;
    can_parameter.trans_fifo_order = DISABLE;

    switch (cfg->mode)
    {
    case RT_CAN_MODE_NORMAL:
        can_parameter.working_mode = CAN_NORMAL_MODE;
        break;
    case RT_CAN_MODE_LISEN:
        can_parameter.working_mode = CAN_SILENT_MODE;
        break;
    case RT_CAN_MODE_LOOPBACK:
        can_parameter.working_mode = CAN_LOOPBACK_MODE;
        break;
    case RT_CAN_MODE_LOOPBACKANLISEN:
        can_parameter.working_mode = CAN_SILENT_LOOPBACK_MODE;
        break;
    }
    can_parameter.working_mode = CAN_NORMAL_MODE;

    /* baud计算 */
    can_parameter.resync_jump_width = can_baud_rate_tab[baud_index].bt_swj;
    can_parameter.time_segment_1 = can_baud_rate_tab[baud_index].bt_bs1;
    can_parameter.time_segment_2 = can_baud_rate_tab[baud_index].bt_bs2;
    can_parameter.prescaler = can_baud_rate_tab[baud_index].bt_psc;
    can_init(can_dev->config->can_periph, &can_parameter);


    /* initialize filter */
    can_filter.filter_number = 0;
    can_filter.filter_mode = CAN_FILTERMODE_MASK;
    can_filter.filter_bits = CAN_FILTERBITS_32BIT;
    can_filter.filter_list_high = 0x0000;
    can_filter.filter_list_low = 0x0000;
    can_filter.filter_mask_high = 0x0000;
    can_filter.filter_mask_low = 0x0000;  
    can_filter.filter_fifo_number = CAN_FIFO0;
    can_filter.filter_enable = ENABLE;
    can_filter_init(&can_filter);

    return RT_EOK;
}

static rt_err_t _can_control(struct rt_can_device *can, int cmd, void *arg)
{
    rt_uint32_t argval;
    RT_ASSERT(can != RT_NULL);
    RT_ASSERT(arg != RT_NULL);

    gd32_can_device *can_dev = rt_container_of(can, gd32_can_device, device);
    uint32_t can_periph = can_dev->config->can_periph;

    switch (cmd)
    {
    case RT_DEVICE_CTRL_CLR_INT:
    {
        argval = (rt_uint32_t)arg;
        if (argval == RT_DEVICE_FLAG_INT_RX)
        {
            if (can_periph == CAN0)
            {
                nvic_irq_disable(USBD_LP_CAN0_RX0_IRQn);
                nvic_irq_disable(CAN0_RX1_IRQn);
            }
#ifdef GD32F10X_CL
            else if (can_periph == CAN1)
            {
                nvic_irq_disable(CAN1_RX0_IRQn);
                nvic_irq_disable(CAN1_RX1_IRQn);
            }
#endif
            can_interrupt_disable(can_periph, CAN_INT_RFNE0);
            can_interrupt_disable(can_periph, CAN_INT_RFF0);
            can_interrupt_disable(can_periph, CAN_INT_RFO0);
            can_interrupt_disable(can_periph, CAN_INT_RFNE1);
            can_interrupt_disable(can_periph, CAN_INT_RFF1);
            can_interrupt_disable(can_periph, CAN_INT_RFO1);
        }
        else if (argval == RT_DEVICE_FLAG_INT_TX)
        {
            if (can_periph == CAN0)
            {
                nvic_irq_disable(USBD_HP_CAN0_TX_IRQn);
            }
#ifdef GD32F10X_CL
            else if (can_periph == CAN1)
            {
                nvic_irq_disable(CAN1_TX_IRQn);
            }
#endif
            can_interrupt_disable(can_periph, CAN_INT_TME);
        }
        else if (argval == RT_DEVICE_CAN_INT_ERR)
        {
            if (can_periph == CAN0)
            {
                nvic_irq_disable(CAN0_EWMC_IRQn);
            }
#ifdef GD32F10X_CL
            else if (can_periph == CAN1)
            {
                nvic_irq_disable(CAN1_EWMC_IRQn);
            }
#endif
            can_interrupt_disable(can_periph, CAN_INT_WERR);
            can_interrupt_disable(can_periph, CAN_INT_PERR);
            can_interrupt_disable(can_periph, CAN_INT_BO);
            can_interrupt_disable(can_periph, CAN_INT_ERRN);
            can_interrupt_disable(can_periph, CAN_INT_ERR);
        }
    } break;
    
    case RT_DEVICE_CTRL_SET_INT:
    {
        argval = (rt_uint32_t)arg;
        if (argval == RT_DEVICE_FLAG_INT_RX)
        {
            if (can_periph == CAN0)
            {
                nvic_irq_enable(USBD_LP_CAN0_RX0_IRQn, 4, 0);
                nvic_irq_enable(CAN0_RX1_IRQn, 4, 0);
            }
#ifdef GD32F10X_CL
            else if (can_periph == CAN1)
            {
                nvic_irq_enable(CAN1_RX0_IRQn, 4, 0);
                nvic_irq_enable(CAN1_RX1_IRQn, 4, 0);
            }
#endif
            can_interrupt_enable(can_periph, CAN_INT_RFNE0);
            can_interrupt_enable(can_periph, CAN_INT_RFF0);
            can_interrupt_enable(can_periph, CAN_INT_RFO0);
            can_interrupt_enable(can_periph, CAN_INT_RFNE1);
            can_interrupt_enable(can_periph, CAN_INT_RFF1);
            can_interrupt_enable(can_periph, CAN_INT_RFO1);
        }
        else if (argval == RT_DEVICE_FLAG_INT_TX)
        {
            if (can_periph == CAN0)
            {
                nvic_irq_enable(USBD_HP_CAN0_TX_IRQn, 4, 0);
            }
#ifdef GD32F10X_CL
            else if (can_periph == CAN1)
            {
                nvic_irq_enable(CAN1_TX_IRQn, 4, 0);
            }
#endif
            can_interrupt_enable(can_periph, CAN_INT_TME);
        }
        else if (argval == RT_DEVICE_CAN_INT_ERR)
        {
            if (can_periph == CAN0)
            {
                nvic_irq_enable(CAN0_EWMC_IRQn, 4, 0);
            }
#ifdef GD32F10X_CL
            else if (can_periph == CAN1)
            {
                nvic_irq_enable(CAN1_EWMC_IRQn, 4, 0);
            }
#endif
            can_interrupt_enable(can_periph, CAN_INT_WERR);
            can_interrupt_enable(can_periph, CAN_INT_PERR);
            can_interrupt_enable(can_periph, CAN_INT_BO);
            can_interrupt_enable(can_periph, CAN_INT_ERRN);
            can_interrupt_enable(can_periph, CAN_INT_ERR);
        }
    } break;

    case RT_CAN_CMD_SET_FILTER:
    {
        struct rt_can_filter_config *filter_cfg = (struct rt_can_filter_config *)arg;
        can_filter_parameter_struct can_filter;

        for (size_t i = 0; i < filter_cfg->count; i++)
        {
            can_filter.filter_number = filter_cfg->items[i].hdr;
            can_filter.filter_mode = filter_cfg->items[i].mode;
            can_filter.filter_bits = CAN_FILTERBITS_32BIT;

            can_filter.filter_list_high = (filter_cfg->items[i].id >> 13) & 0xFFFF;
            can_filter.filter_list_low = ((filter_cfg->items[i].id << 3) |
                                          (filter_cfg->items[i].ide << 2) |
                                          (filter_cfg->items[i].rtr << 1)) & 0xFFFF;
            can_filter.filter_mask_high = (filter_cfg->items[i].mask >> 16) & 0xFFFF;
            can_filter.filter_mask_low = filter_cfg->items[i].mask & 0xFFFF;
            can_filter.filter_fifo_number = CAN_FIFO0;
            can_filter.filter_enable = (ControlStatus)filter_cfg->actived;

            can_filter_init(&can_filter);
        }
    } break;

    case RT_CAN_CMD_SET_MODE:
    {
        argval = (rt_uint32_t)arg;
        if (argval != RT_CAN_MODE_NORMAL &&
            argval != RT_CAN_MODE_LISEN &&
            argval != RT_CAN_MODE_LOOPBACK &&
            argval != RT_CAN_MODE_LOOPBACKANLISEN)
        {
            return -RT_ERROR;
        }
        if (argval != can->config.mode)
        {
            can->config.mode = argval;
            return _can_config(can, &can->config);
        }
    } break;

    case RT_CAN_CMD_SET_BAUD:
    {
        argval = (rt_uint32_t)arg;
        if (argval != CAN1MBaud &&
            argval != CAN800kBaud &&
            argval != CAN500kBaud &&
            argval != CAN250kBaud &&
            argval != CAN125kBaud &&
            argval != CAN100kBaud &&
            argval != CAN50kBaud &&
            argval != CAN20kBaud &&
            argval != CAN10kBaud)
        {
            return -RT_ERROR;
        }
        if (argval != can->config.baud_rate)
        {
            can->config.baud_rate = argval;
            return _can_config(can, &can->config);
        }
    } break;

    case RT_CAN_CMD_SET_PRIV:
    {
        argval = (rt_uint32_t)arg;
        if (argval != RT_CAN_MODE_PRIV && argval != RT_CAN_MODE_NOPRIV)
        {
            return -RT_ERROR;
        }
        if (argval != can->config.privmode)
        {
            can->config.privmode = argval;
            return _can_config(can, &can->config);
        }
    } break;

    case RT_CAN_CMD_GET_STATUS:
    {
        rt_uint32_t errtype = CAN_ERR(can_periph);
        can->status.rcverrcnt = GET_BITS(errtype, 24, 31);
        can->status.snderrcnt = GET_BITS(errtype, 16, 23);;
        can->status.lasterrtype = GET_BITS(errtype, 4, 6);;
        can->status.errcode = GET_BITS(errtype, 0, 2);

        rt_memcpy(arg, &can->status, sizeof(can->status));
    } break;

    default:
        break;
    }

    return RT_EOK;
}

static int _can_sendmsg(struct rt_can_device *can, const void *buf, rt_uint32_t box_num)
{
    can_trasnmit_message_struct transmit_message = {0};

    RT_ASSERT(can != RT_NULL);
    RT_ASSERT(can != buf);

    gd32_can_device *can_dev = rt_container_of(can, gd32_can_device, device);
    uint32_t can_periph = can_dev->config->can_periph;

    struct rt_can_msg *pmsg = (struct rt_can_msg *)buf;
    RT_ASSERT(pmsg->len <= 8);

    if (pmsg->ide == RT_CAN_STDID)
    {
        transmit_message.tx_ff = CAN_FF_STANDARD;   /* format of frame, standard or extended format */
        transmit_message.tx_sfid = pmsg->id;
    }
    else
    {
        transmit_message.tx_ff = CAN_FF_EXTENDED;
        transmit_message.tx_efid = pmsg->id;
    }

    /* type of frame, data or remote */
    transmit_message.tx_ft = (pmsg->rtr == RT_CAN_DTR) ? CAN_FT_DATA : CAN_FT_REMOTE;
    transmit_message.tx_dlen = pmsg->len & 0x0F;

    if (can_message_transmit(can_periph, &transmit_message) == CAN_NOMAILBOX)
        return -RT_ERROR;

    return RT_EOK;
}

static int _can_recvmsg(struct rt_can_device *can, void *buf, rt_uint32_t fifo)
{
    can_receive_message_struct receive_message = {0};

    RT_ASSERT(can != RT_NULL);
    RT_ASSERT(buf != RT_NULL);

    gd32_can_device *can_dev = rt_container_of(can, gd32_can_device, device);
    uint32_t can_periph = can_dev->config->can_periph;
    struct rt_can_msg *pmsg = (struct rt_can_msg *)buf;

    can_message_receive(can_periph, fifo, &receive_message);

    if (receive_message.rx_ff == CAN_FF_STANDARD)
    {
        pmsg->ide = RT_CAN_STDID;
        pmsg->id = receive_message.rx_sfid;
    }
    else
    {
        pmsg->ide = RT_CAN_EXTID;
        pmsg->id = receive_message.rx_efid;
    }

    pmsg->rtr = (receive_message.rx_ft == CAN_FT_DATA) ? RT_CAN_DTR : RT_CAN_RTR;
    pmsg->len = receive_message.rx_dlen;
    pmsg->hdr = (receive_message.rx_fi + 1) >> 1;

    rt_memcpy(pmsg->data, receive_message.rx_data, 8);

    return RT_EOK;
}

/**
 * @brief can接收中断处理
 * @param[in]  can      can设备
 * @param[in]  fifo     接收FIFO（0/1）
 */
static void _can_rx_isr(struct rt_can_device *can, rt_uint32_t fifo)
{
    RT_ASSERT(can);

    gd32_can_device *can_dev = rt_container_of(can, gd32_can_device, device);
    uint32_t can_periph = can_dev->config->can_periph;

    switch (fifo)
    {
    case CAN_FIFO0:
    {
        if (can_receive_message_length_get(can_periph, fifo) && can_interrupt_flag_get(can_periph, CAN_INT_FLAG_RFL0))
        {
            can_interrupt_flag_clear(can_periph, CAN_INT_FLAG_RFL0);
            rt_hw_can_isr(can, RT_CAN_EVENT_RX_IND | fifo << 8);
        }

        /* Check FULL flag for FIFO0 */
        if (can_flag_get(can_periph, CAN_FLAG_RFF0) && can_interrupt_flag_get(can_periph, CAN_INT_FLAG_RFF0))
        {
            can_flag_clear(can_periph, CAN_FLAG_RFF0);
            can_interrupt_flag_clear(can_periph, CAN_INT_FLAG_RFF0);
        }

        /* Check Overrun flag for FIFO0 */
        if (can_flag_get(can_periph, CAN_FLAG_RFO0) && can_interrupt_flag_get(can_periph, CAN_INT_FLAG_RFO0))
        {
            can_flag_clear(can_periph, CAN_FLAG_RFO0);
            can_interrupt_flag_clear(can_periph, CAN_INT_FLAG_RFO0);
            rt_hw_can_isr(can, RT_CAN_EVENT_RXOF_IND | fifo << 8);
        }
    } break;

    case CAN_FIFO1:
    {
        if (can_receive_message_length_get(can_periph, fifo) && can_interrupt_flag_get(can_periph, CAN_INT_FLAG_RFL1))
        {
            rt_hw_can_isr(can, RT_CAN_EVENT_RX_IND | fifo << 8);
        }

        /* Check FULL flag for FIFO1 */
        if (can_flag_get(can_periph, CAN_FLAG_RFF1) && can_interrupt_flag_get(can_periph, CAN_INT_FLAG_RFF1))
        {
            can_flag_clear(can_periph, CAN_FLAG_RFF1);
            can_interrupt_flag_clear(can_periph, CAN_INT_FLAG_RFF1);
        }

        /* Check Overrun flag for FIFO1 */
        if (can_flag_get(can_periph, CAN_FLAG_RFO1) && can_interrupt_flag_get(can_periph, CAN_INT_FLAG_RFO1))
        {
            can_flag_clear(can_periph, CAN_FLAG_RFO1);
            can_interrupt_flag_clear(can_periph, CAN_INT_FLAG_RFO1);
            rt_hw_can_isr(can, RT_CAN_EVENT_RXOF_IND | fifo << 8);
        }
    } break;
    
    default:
        break;
    }
}

/**
 * @brief can发送中断处理
 * @param[in]  can      can设备
 */
static void _can_tx_isr(struct rt_can_device *can)
{
    RT_ASSERT(can);

    gd32_can_device *can_dev = rt_container_of(can, gd32_can_device, device);
    uint32_t can_periph = can_dev->config->can_periph;

    /* mailbox 0 transmit finished interrupt flag */
    if (can_interrupt_flag_get(can_periph, CAN_INT_FLAG_MTF0))
    {
        can_interrupt_flag_clear(can_periph, CAN_INT_FLAG_MTF0);
        if (can_flag_get(can_periph, CAN_FLAG_MTFNERR0))  /* finished with no error */
        {
            can_flag_clear(can_periph, CAN_FLAG_MTFNERR0);
            rt_hw_can_isr(can, RT_CAN_EVENT_TX_DONE | 0 << 8);
        }
        else
        {
            rt_hw_can_isr(can, RT_CAN_EVENT_TX_FAIL | 0 << 8);
        }
    }
    else if (can_interrupt_flag_get(can_periph, CAN_INT_FLAG_MTF1))
    {
        can_interrupt_flag_clear(can_periph, CAN_INT_FLAG_MTF1);
        if (can_flag_get(can_periph, CAN_FLAG_MTFNERR1))  /* finished with no error */
        {
            can_flag_clear(can_periph, CAN_FLAG_MTFNERR1);
            rt_hw_can_isr(can, RT_CAN_EVENT_TX_DONE | 0 << 8);
        }
        else
        {
            rt_hw_can_isr(can, RT_CAN_EVENT_TX_FAIL | 0 << 8);
        }
    }
    else if (can_interrupt_flag_get(can_periph, CAN_INT_FLAG_MTF2))
    {
        can_interrupt_flag_clear(can_periph, CAN_INT_FLAG_MTF2);
        if (can_flag_get(can_periph, CAN_FLAG_MTFNERR2))  /* finished with no error */
        {
            can_flag_clear(can_periph, CAN_FLAG_MTFNERR2);
            rt_hw_can_isr(can, RT_CAN_EVENT_TX_DONE | 0 << 8);
        }
        else
        {
            rt_hw_can_isr(can, RT_CAN_EVENT_TX_FAIL | 0 << 8);
        }
    }
    else
    {
        rt_hw_can_isr(can, RT_CAN_EVENT_TX_FAIL | 0 << 8);
    }
}

/**
 * @brief can错误中断处理
 * @param[in]  can      can设备
 */
static void _can_err_isr(struct rt_can_device *can)
{
    RT_ASSERT(can);

    gd32_can_device *can_dev = rt_container_of(can, gd32_can_device, device);
    uint32_t can_periph = can_dev->config->can_periph;

    can_error_enum errtype = can_error_get(can_periph);
    switch (errtype)
    {
    case CAN_ERROR_ACK:
    {
        if (can_flag_get(can_periph, CAN_FLAG_MTE0))
            rt_hw_can_isr(can, RT_CAN_EVENT_TX_FAIL | 0 << 8);
        else if (can_flag_get(can_periph, CAN_FLAG_MTE1))
            rt_hw_can_isr(can, RT_CAN_EVENT_TX_FAIL | 1 << 8);
        else if (can_flag_get(can_periph, CAN_FLAG_MTE2))
            rt_hw_can_isr(can, RT_CAN_EVENT_TX_FAIL | 2 << 8);
    } break;

    case CAN_ERROR_FILL:
        /* code */
        break;

    case CAN_ERROR_FORMATE:
        can->status.formaterrcnt++;
        break;

    case CAN_ERROR_BITRECESSIVE:
    case CAN_ERROR_BITDOMINANTER:
        can->status.biterrcnt++;
        break;

    case CAN_ERROR_CRC:
        can->status.crcerrcnt++;
        break;
    
    default:
        break;
    }

    if (can_interrupt_flag_get(can_periph, CAN_INT_FLAG_ERRIF))
    {
        can_interrupt_flag_clear(can_periph, CAN_INT_FLAG_ERRIF);
    }
    if (can_interrupt_flag_get(can_periph, CAN_INT_FLAG_ERRN))
    {
        can_interrupt_flag_clear(can_periph, CAN_INT_FLAG_ERRN);
    }
    if (can_interrupt_flag_get(can_periph, CAN_INT_FLAG_BOERR))
    {
        can_interrupt_flag_clear(can_periph, CAN_INT_FLAG_BOERR);
    }
    if (can_interrupt_flag_get(can_periph, CAN_INT_FLAG_PERR))
    {
        can_interrupt_flag_clear(can_periph, CAN_INT_FLAG_PERR);
    }
    if (can_interrupt_flag_get(can_periph, CAN_INT_FLAG_WERR))
    {
        can_interrupt_flag_clear(can_periph, CAN_INT_FLAG_WERR);
    }
}

#ifdef BSP_USING_CAN0
void USBD_HP_CAN0_TX_IRQHandler(void)
{
    rt_interrupt_enter();
    _can_tx_isr(&can_devs[0].device);
    rt_interrupt_leave();
}

void USBD_LP_CAN0_RX0_IRQHandler(void)
{
    rt_interrupt_enter();
    _can_rx_isr(&can_devs[0].device, CAN_FIFO0);
    rt_interrupt_leave();
}

void CAN0_RX1_IRQHandler(void)
{
    rt_interrupt_enter();
    _can_rx_isr(&can_devs[0].device, CAN_FIFO1);
    rt_interrupt_leave();
}

void CAN0_EWMC_IRQHandler(void)
{
    rt_interrupt_enter();
    _can_err_isr(&can_devs[0].device);
    rt_interrupt_leave();
}
#endif


static const struct rt_can_ops _can_ops =
{
    _can_config,
    _can_control,
    _can_sendmsg,
    _can_recvmsg,
};

int rt_hw_can_init(void)
{
    rt_err_t result = 0;
    rt_size_t dev_num = sizeof(can_devs) / sizeof(gd32_can_device);

    struct can_configure config = CANDEFAULTCONFIG;
    config.privmode = RT_CAN_MODE_NOPRIV;
    config.ticks = 50;
#ifdef RT_CAN_USING_HDR /* 硬件过滤 */
    config.maxhdr = 14;
#ifdef BSP_USING_CAN1
    config.maxhdr = 28;
#endif
#endif

    for (int i = 0; i < dev_num; i++)
    {
        can_devs[i].config = &can_dev_config[i];
        can_devs[i].device.config = config;

        gd32_can_gpio_init(&can_dev_config[i]);

        result = rt_hw_can_register(&can_devs[i].device, can_dev_config[i].name, &_can_ops, NULL);
        RT_ASSERT(result == RT_EOK);
    }

    return result;
}
INIT_BOARD_EXPORT(rt_hw_can_init);
#endif

/*********** (C) COPYRIGHT 2022 FiberHome *****END OF FILE****/
