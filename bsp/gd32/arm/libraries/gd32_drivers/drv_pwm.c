/**
 * @file drv_pwm.c
 * @brief GD32 PWM-RTT适配驱动
 * @note 仅适配特定时钟配置
 * @author wangh (wanghuan3037@fiberhome.com)
 * @version 1.0
 * @date 2022-03-24
 * @copyright Copyright (c) 2022  烽火通信
 */ 
#include <board.h>
#ifdef RT_USING_PWM
#include "pwm_config.h"

#define DBG_TAG "drv.pwm"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define MAX_PERIOD  65535
#define MIN_PERIOD  3
#define MIN_PULSE   2

enum
{
#ifdef BSP_USING_PWM0
    PWM0_INDEX,
#endif
#ifdef BSP_USING_PWM1
    PWM1_INDEX,
#endif
#ifdef BSP_USING_PWM2
    PWM2_INDEX,
#endif
#ifdef BSP_USING_PWM3
    PWM3_INDEX,
#endif
#ifdef BSP_USING_PWM4
    PWM4_INDEX,
#endif
#ifdef BSP_USING_PWM5
    PWM5_INDEX,
#endif
#ifdef BSP_USING_PWM6
    PWM6_INDEX,
#endif
#ifdef BSP_USING_PWM7
    PWM7_INDEX,
#endif
#ifdef BSP_USING_PWM8
    PWM8_INDEX,
#endif
#ifdef BSP_USING_PWM9
    PWM9_INDEX,
#endif
#ifdef BSP_USING_PWM10
    PWM10_INDEX,
#endif
#ifdef BSP_USING_PWM11
    PWM11_INDEX,
#endif
#ifdef BSP_USING_PWM12
    PWM12_INDEX,
#endif
#ifdef BSP_USING_PWM13
    PWM13_INDEX,
#endif
};

/** @brief gd32 pwm设备结构体 */
typedef struct 
{
    struct rt_device_pwm device;
    uint32_t timer_periph;
    rt_uint8_t channel;
    char *name;
} gd32_pwm_device;

static gd32_pwm_device pwm_obj[] = 
{
#ifdef BSP_USING_PWM0
    PWM0_CONFIG,
#endif

#ifdef BSP_USING_PWM1
    PWM1_CONFIG,
#endif

#ifdef BSP_USING_PWM2
    PWM2_CONFIG,
#endif

#ifdef BSP_USING_PWM3
    PWM3_CONFIG,
#endif

#ifdef BSP_USING_PWM4
    PWM4_CONFIG,
#endif

#ifdef BSP_USING_PWM5
    PWM5_CONFIG,
#endif

#ifdef BSP_USING_PWM6
    PWM6_CONFIG,
#endif

#ifdef BSP_USING_PWM7
    PWM7_CONFIG,
#endif

#ifdef BSP_USING_PWM8
    PWM8_CONFIG,
#endif

#ifdef BSP_USING_PWM9
    PWM9_CONFIG,
#endif

#ifdef BSP_USING_PWM10
    PWM10_CONFIG,
#endif

#ifdef BSP_USING_PWM11
    PWM11_CONFIG,
#endif

#ifdef BSP_USING_PWM12
    PWM12_CONFIG,
#endif

#ifdef BSP_USING_PWM13
    PWM13_CONFIG,
#endif
};

static void pwm_get_channel(void)
{
#ifdef BSP_USING_PWM0_CH1
    pwm_obj[PWM0_INDEX].channel |= 1 << 0;
#endif
#ifdef BSP_USING_PWM0_CH2
    pwm_obj[PWM0_INDEX].channel |= 1 << 1;
#endif
#ifdef BSP_USING_PWM0_CH3
    pwm_obj[PWM0_INDEX].channel |= 1 << 2;
#endif
#ifdef BSP_USING_PWM0_CH4
    pwm_obj[PWM0_INDEX].channel |= 1 << 3;
#endif
#ifdef BSP_USING_PWM1_CH1
    pwm_obj[PWM1_INDEX].channel |= 1 << 0;
#endif
#ifdef BSP_USING_PWM1_CH2
    pwm_obj[PWM1_INDEX].channel |= 1 << 1;
#endif
#ifdef BSP_USING_PWM1_CH3
    pwm_obj[PWM1_INDEX].channel |= 1 << 2;
#endif
#ifdef BSP_USING_PWM1_CH4
    pwm_obj[PWM1_INDEX].channel |= 1 << 3;
#endif
#ifdef BSP_USING_PWM2_CH1
    pwm_obj[PWM2_INDEX].channel |= 1 << 0;
#endif
#ifdef BSP_USING_PWM2_CH2
    pwm_obj[PWM2_INDEX].channel |= 1 << 1;
#endif
#ifdef BSP_USING_PWM2_CH3
    pwm_obj[PWM2_INDEX].channel |= 1 << 2;
#endif
#ifdef BSP_USING_PWM2_CH4
    pwm_obj[PWM2_INDEX].channel |= 1 << 3;
#endif
#ifdef BSP_USING_PWM3_CH1
    pwm_obj[PWM3_INDEX].channel |= 1 << 0;
#endif
#ifdef BSP_USING_PWM3_CH2
    pwm_obj[PWM3_INDEX].channel |= 1 << 1;
#endif
#ifdef BSP_USING_PWM3_CH3
    pwm_obj[PWM3_INDEX].channel |= 1 << 2;
#endif
#ifdef BSP_USING_PWM3_CH4
    pwm_obj[PWM3_INDEX].channel |= 1 << 3;
#endif
#ifdef BSP_USING_PWM4_CH1
    pwm_obj[PWM4_INDEX].channel |= 1 << 0;
#endif
#ifdef BSP_USING_PWM4_CH2
    pwm_obj[PWM4_INDEX].channel |= 1 << 1;
#endif
#ifdef BSP_USING_PWM4_CH3
    pwm_obj[PWM4_INDEX].channel |= 1 << 2;
#endif
#ifdef BSP_USING_PWM4_CH4
    pwm_obj[PWM4_INDEX].channel |= 1 << 3;
#endif
#ifdef BSP_USING_PWM5_CH1
    pwm_obj[PWM5_INDEX].channel |= 1 << 0;
#endif
#ifdef BSP_USING_PWM5_CH2
    pwm_obj[PWM5_INDEX].channel |= 1 << 1;
#endif
#ifdef BSP_USING_PWM5_CH3
    pwm_obj[PWM5_INDEX].channel |= 1 << 2;
#endif
#ifdef BSP_USING_PWM5_CH4
    pwm_obj[PWM5_INDEX].channel |= 1 << 3;
#endif
#ifdef BSP_USING_PWM6_CH1
    pwm_obj[PWM6_INDEX].channel |= 1 << 0;
#endif
#ifdef BSP_USING_PWM6_CH2
    pwm_obj[PWM6_INDEX].channel |= 1 << 1;
#endif
#ifdef BSP_USING_PWM6_CH3
    pwm_obj[PWM6_INDEX].channel |= 1 << 2;
#endif
#ifdef BSP_USING_PWM6_CH4
    pwm_obj[PWM6_INDEX].channel |= 1 << 3;
#endif
#ifdef BSP_USING_PWM7_CH1
    pwm_obj[PWM7_INDEX].channel |= 1 << 0;
#endif
#ifdef BSP_USING_PWM7_CH2
    pwm_obj[PWM7_INDEX].channel |= 1 << 1;
#endif
#ifdef BSP_USING_PWM7_CH3
    pwm_obj[PWM7_INDEX].channel |= 1 << 2;
#endif
#ifdef BSP_USING_PWM7_CH4
    pwm_obj[PWM7_INDEX].channel |= 1 << 3;
#endif
#ifdef BSP_USING_PWM8_CH1
    pwm_obj[PWM8_INDEX].channel |= 1 << 0;
#endif
#ifdef BSP_USING_PWM8_CH2
    pwm_obj[PWM8_INDEX].channel |= 1 << 1;
#endif
#ifdef BSP_USING_PWM8_CH3
    pwm_obj[PWM8_INDEX].channel |= 1 << 2;
#endif
#ifdef BSP_USING_PWM8_CH4
    pwm_obj[PWM8_INDEX].channel |= 1 << 3;
#endif
#ifdef BSP_USING_PWM9_CH1
    pwm_obj[PWM9_INDEX].channel |= 1 << 0;
#endif
#ifdef BSP_USING_PWM9_CH2
    pwm_obj[PWM9_INDEX].channel |= 1 << 1;
#endif
#ifdef BSP_USING_PWM9_CH3
    pwm_obj[PWM9_INDEX].channel |= 1 << 2;
#endif
#ifdef BSP_USING_PWM9_CH4
    pwm_obj[PWM9_INDEX].channel |= 1 << 3;
#endif
#ifdef BSP_USING_PWM12_CH1
    pwm_obj[PWM12_INDEX].channel |= 1 << 0;
#endif
#ifdef BSP_USING_PWM12_CH2
    pwm_obj[PWM12_INDEX].channel |= 1 << 1;
#endif
}

RT_WEAK void gd32_pwm_pin_init(uint32_t timer_periph)
{
    /* PWM 引脚初始化 */
}

static rt_err_t drv_pwm_init(gd32_pwm_device *device)
{
    timer_oc_parameter_struct timer_ocintpara;
    timer_parameter_struct timer_initpara;

    gd32_pwm_pin_init(device->timer_periph);

    timer_deinit(device->timer_periph);
    /* TIMER1 configuration */
    timer_initpara.prescaler = 0;
    timer_initpara.alignedmode = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection = TIMER_COUNTER_UP;
    timer_initpara.period = 0;
    timer_initpara.clockdivision = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(device->timer_periph, &timer_initpara);

    /* OC_CH configuration in PWM mode1 */
    timer_ocintpara.outputstate = TIMER_CCX_ENABLE;
    timer_ocintpara.outputnstate = TIMER_CCXN_DISABLE;
    timer_ocintpara.ocpolarity = TIMER_OC_POLARITY_HIGH;
    timer_ocintpara.ocnpolarity = TIMER_OCN_POLARITY_HIGH;
    timer_ocintpara.ocidlestate = TIMER_OC_IDLE_STATE_LOW;
    timer_ocintpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;

    if (device->channel & 0x01)
    {
        timer_channel_output_config(device->timer_periph, TIMER_CH_0, &timer_ocintpara);
        timer_channel_output_pulse_value_config(device->timer_periph, TIMER_CH_0, 0);
        timer_channel_output_mode_config(device->timer_periph, TIMER_CH_0, TIMER_OC_MODE_PWM0);
        timer_channel_output_shadow_config(device->timer_periph, TIMER_CH_0, TIMER_OC_SHADOW_DISABLE);
    }

    if (device->channel & 0x02)
    {
        timer_channel_output_config(device->timer_periph, TIMER_CH_1, &timer_ocintpara);
        timer_channel_output_pulse_value_config(device->timer_periph, TIMER_CH_1, 0);
        timer_channel_output_mode_config(device->timer_periph, TIMER_CH_1, TIMER_OC_MODE_PWM0);
        timer_channel_output_shadow_config(device->timer_periph, TIMER_CH_1, TIMER_OC_SHADOW_DISABLE);
    }

    if (device->channel & 0x04)
    {
        timer_channel_output_config(device->timer_periph, TIMER_CH_2, &timer_ocintpara);
        timer_channel_output_pulse_value_config(device->timer_periph, TIMER_CH_2, 0);
        timer_channel_output_mode_config(device->timer_periph, TIMER_CH_2, TIMER_OC_MODE_PWM0);
        timer_channel_output_shadow_config(device->timer_periph, TIMER_CH_2, TIMER_OC_SHADOW_DISABLE);
    }

    if (device->channel & 0x08)
    {
        timer_channel_output_config(device->timer_periph, TIMER_CH_3, &timer_ocintpara);
        timer_channel_output_pulse_value_config(device->timer_periph, TIMER_CH_3, 0);
        timer_channel_output_mode_config(device->timer_periph, TIMER_CH_3, TIMER_OC_MODE_PWM0);
        timer_channel_output_shadow_config(device->timer_periph, TIMER_CH_3, TIMER_OC_SHADOW_DISABLE);
    }

    timer_auto_reload_shadow_enable(device->timer_periph);
    timer_enable(device->timer_periph);

    return RT_EOK;
}

static rt_err_t drv_pwm_enable(uint32_t timer_periph, struct rt_pwm_configuration *configuration, rt_bool_t enable)
{
    uint16_t channel = configuration->channel - 1;  /* TIMER_CH_0 ~ TIMER_CH_3 */
    timer_channel_output_state_config(timer_periph, channel, (enable) ? TIMER_CCX_ENABLE : TIMER_CCX_DISABLE);

    return RT_EOK;
}

static rt_err_t drv_pwm_get(uint32_t timer_periph, struct rt_pwm_configuration *configuration)
{
    uint32_t tim_clock = SystemCoreClock;   /* 适用于 APB1=AHB/2  APB2=AHB/1 */

    /* Convert nanosecond to frequency and duty cycle. 1s = 1 * 1000 * 1000 * 1000 ns */
    tim_clock /= 1000000UL;
    configuration->period = (TIMER_CAR(timer_periph) + 1) * (TIMER_PSC(timer_periph) + 1) * 1000UL / tim_clock;
    configuration->pulse = (REG32((timer_periph) + 0x34U + (configuration->channel - 1) * 4) + 1) * (TIMER_PSC(timer_periph) + 1) * 1000UL / tim_clock;

    return RT_EOK;
}

static rt_err_t drv_pwm_set(uint32_t timer_periph, struct rt_pwm_configuration *configuration)
{
    uint32_t tim_clock = SystemCoreClock;    /* 适用于 APB1=AHB/2  APB2=AHB/1 */
    uint32_t period, pulse, psc;

    /* Convert nanosecond to frequency and duty cycle. 1s = 1 * 1000 * 1000 * 1000 ns */
    tim_clock /= 1000000UL;
    period = (unsigned long long)configuration->period * tim_clock / 1000ULL;
    psc = period / MAX_PERIOD + 1;  /* 计算最小分频 */
    period = period / psc;
    timer_prescaler_config(timer_periph, psc - 1, TIMER_PSC_RELOAD_UPDATE);

    if (period < MIN_PERIOD)
        period = MIN_PERIOD;
    timer_autoreload_value_config(timer_periph, period - 1);

    pulse = (unsigned long long)configuration->pulse * tim_clock / psc / 1000ULL;
    if (pulse < MIN_PULSE)
    {
        pulse = MIN_PULSE;
    }
    else if (pulse > period)
    {
        pulse = period;
    }
    timer_channel_output_pulse_value_config(timer_periph, configuration->channel - 1, pulse - 1);
    timer_counter_value_config(timer_periph, 0);

    timer_event_software_generate(timer_periph, TIMER_EVENT_SRC_UPG);

    return RT_EOK;
}

static rt_err_t drv_pwm_control(struct rt_device_pwm *device, int cmd, void *arg)
{
    RT_ASSERT(device != RT_NULL);
    RT_ASSERT(arg != RT_NULL);

    struct rt_pwm_configuration *configuration = (struct rt_pwm_configuration *)arg;
    gd32_pwm_device *pwm_dev = rt_container_of(device, gd32_pwm_device, device);

    switch (cmd)
    {
    case PWM_CMD_ENABLE:
        return drv_pwm_enable(pwm_dev->timer_periph, configuration, RT_TRUE);
    case PWM_CMD_DISABLE:
        return drv_pwm_enable(pwm_dev->timer_periph, configuration, RT_FALSE);
    case PWM_CMD_SET:
        return drv_pwm_set(pwm_dev->timer_periph, configuration);
    case PWM_CMD_GET:
        return drv_pwm_get(pwm_dev->timer_periph, configuration);
    default:
        return RT_EINVAL;
    }
}

static struct rt_pwm_ops drv_ops =
{
    .control = drv_pwm_control
};

static int rt_hw_pwm_init(void)
{
    pwm_get_channel();

    for (uint8_t i = 0; i < sizeof(pwm_obj) / sizeof(pwm_obj[0]); i++)
    {
        /* pwm init */
        if (drv_pwm_init(&pwm_obj[i]) != RT_EOK)
        {
            LOG_E("%s init failed", pwm_obj[i].name);
            return -RT_ERROR;
        }
        else
        {
            LOG_D("%s init success", pwm_obj[i].name);

            /* register pwm device */
            if (rt_device_pwm_register(&pwm_obj[i].device, pwm_obj[i].name, &drv_ops, NULL) == RT_EOK)
            {
                LOG_D("%s register success", pwm_obj[i].name);
            }
            else
            {
                LOG_E("%s register failed", pwm_obj[i].name);
                return -RT_ERROR;
            }
        }
    }

    return RT_EOK;
}
INIT_DEVICE_EXPORT(rt_hw_pwm_init);
#endif  /* RT_USING_PWM */

/*********** (C) COPYRIGHT 2022 FiberHome *****END OF FILE****/
