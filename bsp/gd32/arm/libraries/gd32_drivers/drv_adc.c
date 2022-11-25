/**
 * @file drv_adc.c
 * @brief GD32 ADC-RTT适配驱动
 * @author wangh (wanghuan3037@fiberhome.com)
 * @version 1.0
 * @date 2022-02-23
 * @copyright Copyright (c) 2022  烽火通信
 */
#include <board.h>

#ifdef RT_USING_ADC

#define DBG_TAG "drv.adc"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define MAX_EXTERN_ADC_CHANNEL  16

typedef struct
{
    struct rt_adc_device adc_dev;
    char name[8];
    rt_base_t adc_pins[16];
    void *private_data;
} gd32_adc_device;

static gd32_adc_device g_gd32_devs[] = {
#ifdef BSP_USING_ADC0
    {
        .name = "adc0",
        .adc_pins = {
            GET_PIN(A, 0), GET_PIN(A, 1), GET_PIN(A, 2), GET_PIN(A, 3),
            GET_PIN(A, 4), GET_PIN(A, 5), GET_PIN(A, 6), GET_PIN(A, 7),
            GET_PIN(B, 0), GET_PIN(B, 1), GET_PIN(C, 0), GET_PIN(C, 1),
            GET_PIN(C, 2), GET_PIN(C, 3), GET_PIN(C, 4), GET_PIN(C, 5),
        },
        .private_data = (void *)ADC0,
    },
#endif

#ifdef BSP_USING_ADC1
    {
        .name = "adc1",
        .adc_pins = {
            GET_PIN(A, 0), GET_PIN(A, 1), GET_PIN(A, 2), GET_PIN(A, 3),
            GET_PIN(A, 4), GET_PIN(A, 5), GET_PIN(A, 6), GET_PIN(A, 7),
            GET_PIN(B, 0), GET_PIN(B, 1), GET_PIN(C, 0), GET_PIN(C, 1),
            GET_PIN(C, 2), GET_PIN(C, 3), GET_PIN(C, 4), GET_PIN(C, 5),
        },
        .private_data = (void *)ADC1,
    },
#endif
};

/**
 * @brief 初始化ADC引脚
 * @param[in]  pin      引脚编号
 */
static void gd32_adc_gpio_init(rt_base_t pin)
{
    rcu_periph_clock_enable((rcu_periph_enum)PIN_GDRCU(pin));
    gpio_init(PIN_GDPORT(pin), GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, PIN_GDPIN(pin));
}

/**
 * @brief adc通道使能
 * @param[in]  device   adc设备
 * @param[in]  channel  adc通道
 * @param[in]  enabled  0：禁止，非0：使能
 * @return rt_err_t 0：成功，非0：失败
 */
static rt_err_t gd32_adc_enabled(struct rt_adc_device *device, rt_uint32_t channel, rt_bool_t enabled)
{
    if ((device == NULL) || (channel >= MAX_EXTERN_ADC_CHANNEL))
    {
        LOG_E("invalid channel\r\n");
        return -RT_EINVAL;
    }

    gd32_adc_device *gd32_adc = (gd32_adc_device *)device;
    uint32_t adc_periph = (uint32_t )(device->parent.user_data);

    if (enabled == ENABLE)
    {
        gd32_adc_gpio_init(gd32_adc->adc_pins[channel]);
        adc_deinit(adc_periph);
        adc_channel_length_config(adc_periph, ADC_REGULAR_CHANNEL, 1);
        adc_data_alignment_config(adc_periph, ADC_DATAALIGN_RIGHT);
        adc_external_trigger_source_config(adc_periph, ADC_REGULAR_CHANNEL, ADC0_1_2_EXTTRIG_INSERTED_NONE);
        adc_external_trigger_config(adc_periph, ADC_REGULAR_CHANNEL, ENABLE);
        adc_special_function_config(adc_periph, ADC_SCAN_MODE, DISABLE);       /* 模数转换工作在单通道模式 */
        adc_special_function_config(adc_periph, ADC_CONTINUOUS_MODE, DISABLE); /* 模数转换工作在单次转换模式 */
        adc_discontinuous_mode_config(adc_periph, ADC_REGULAR_CHANNEL, 1);     /* 顺序进行规则转换的ADC通道的数目 */
        adc_enable(adc_periph);
    }
    else
    {
        adc_disable(adc_periph);
    }

    return 0;
}

/**
 * @brief adc转换
 */
static rt_err_t gd32_adc_convert(struct rt_adc_device *device, rt_uint32_t channel, rt_uint32_t *value)
{
    if ((device == NULL) || (channel >= MAX_EXTERN_ADC_CHANNEL) || (value == NULL))
    {
        LOG_E("invalid param\r\n");
        return -RT_EINVAL;
    }

    uint32_t timeout = 50000;
    uint32_t adc_periph = (uint32_t )(device->parent.user_data);

    adc_flag_clear(adc_periph, ADC_FLAG_EOC);
    adc_regular_channel_config(adc_periph, 0, channel, ADC_SAMPLETIME_239POINT5);
    adc_software_trigger_enable(adc_periph, ADC_REGULAR_CHANNEL);
    while (adc_flag_get(adc_periph, ADC_FLAG_EOC) != SET)
    {
        if ((timeout--) == 0) break;
    } /* 等待转换结束 */
    *value = adc_regular_data_read(adc_periph);

    return 0;
}

static struct rt_adc_ops g_gd32_adc_ops = {
    .enabled = gd32_adc_enabled,
    .convert = gd32_adc_convert,
};

static int rt_hw_adc_init(void)
{
    int ret, i = 0;

#ifdef BSP_USING_ADC0
    rcu_periph_clock_enable(RCU_ADC0);
#endif

#ifdef BSP_USING_ADC1
    rcu_periph_clock_enable(RCU_ADC1);
#endif

    rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV6);    /* 72/6 = 12MHz  need < 14MHZ */
    adc_mode_config(ADC_MODE_FREE);

    for (; i < sizeof(g_gd32_devs) / sizeof(gd32_adc_device); i++)
    {
        ret = rt_hw_adc_register(&g_gd32_devs[i].adc_dev,
                                 (const char *)g_gd32_devs[i].name,
                                 &g_gd32_adc_ops, (void *)g_gd32_devs[i].private_data);
        if (ret != RT_EOK)
        {
            /* TODO err handler */
            LOG_E("failed register %s, err=%d\r\n", g_gd32_devs[i].name, ret);
        }
    }

    return ret;
}
INIT_BOARD_EXPORT(rt_hw_adc_init);
#endif

/*********** (C) COPYRIGHT 2022 FiberHome *****END OF FILE****/
