/**
 * @file drv_dac.c
 * @brief GD32 DAC-RTT适配驱动
 * @author wangh (wanghuan3037@fiberhome.com)
 * @version 1.0
 * @date 2022-03-17
 * @copyright Copyright (c) 2022  烽火通信
 */ 
#include <board.h>

#ifdef RT_USING_DAC

#define DBG_TAG "drv.dac"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

typedef struct
{
    struct rt_dac_device device;
    char name[8];
    rt_base_t dac_pins[2];
} gd32_dac_device;

static gd32_dac_device dac_devs[] = {
#ifdef BSP_USING_DAC0
    {
        .name = "dac0",
        .dac_pins = {GET_PIN(A, 4), GET_PIN(A, 5)},
    },
#endif

#ifdef BSP_USING_DAC1
    {
        .name = "dac1",
        .dac_pins = {GET_PIN(A, 4), GET_PIN(A, 5)},
    },
#endif
};

/**
 * @brief 初始化ADC引脚
 * @param[in]  pin      引脚编号
 */
static void gd32_dac_gpio_init(rt_base_t pin)
{
    rcu_periph_clock_enable((rcu_periph_enum)PIN_GDRCU(pin));
    gpio_init(PIN_GDPORT(pin), GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, PIN_GDPIN(pin));
}


static rt_err_t gd32_dac_enabled(struct rt_dac_device *device, rt_uint32_t channel)
{
    if (channel > 1)
    {
        LOG_E("invalid param.\r\n");
        return -RT_EINVAL;
    }

    gd32_dac_device *gd32_dac = rt_container_of(device, gd32_dac_device, device);
    gd32_dac_gpio_init(gd32_dac->dac_pins[channel]);

    dac_trigger_source_config(channel, DAC_TRIGGER_SOFTWARE);
    dac_trigger_enable(channel);
    dac_wave_mode_config(channel, DAC_WAVE_DISABLE);
    dac_output_buffer_disable(channel);
    dac_enable(channel);

    return RT_EOK;
}

static rt_err_t gd32_dac_disabled(struct rt_dac_device *device, rt_uint32_t channel)
{
    if (channel > 1)
    {
        LOG_E("invalid param.\r\n");
        return -RT_EINVAL;
    }

    dac_disable(channel);

    return RT_EOK;
}

static rt_err_t gd32_set_dac_value(struct rt_dac_device *device, rt_uint32_t channel, rt_uint32_t *value)
{
    if ((channel > 1) || (value == NULL))
    {
        LOG_E("invalid param.\r\n");
        return -RT_EINVAL;
    }

    dac_data_set(channel, DAC_ALIGN_12B_R, *value);
    dac_software_trigger_enable(channel);

    return RT_EOK;
}

static const struct rt_dac_ops dac_ops =
{
    .disabled = gd32_dac_disabled,
    .enabled  = gd32_dac_enabled,
    .convert  = gd32_set_dac_value,
};

static int rt_hw_dac_init(void)
{
    int ret, i;

    rcu_periph_clock_enable(RCU_DAC);

    for (i = 0; i < sizeof(dac_devs) / sizeof(gd32_dac_device); i++)
    {
        ret = rt_hw_dac_register(&dac_devs[i].device, (const char *)dac_devs[i].name, &dac_ops, NULL);
        if (ret != RT_EOK)
        {
            LOG_E("failed register %s, err=%d\r\n", g_gd32_devs[i].name, ret);
        }
    }

    return ret;
}
INIT_DEVICE_EXPORT(rt_hw_dac_init);
#endif  /* RT_USING_DAC */


/*********** (C) COPYRIGHT 2022 FiberHome *****END OF FILE****/
