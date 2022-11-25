/**
 * @file drv_rtc.c
 * @brief GD32 RTC-RTT适配驱动
 * @note alarm暂不支持
 * @author wangh (wanghuan3037@fiberhome.com)
 * @version 1.0
 * @date 2022-03-22
 * @copyright Copyright (c) 2022  烽火通信
 */
#include <board.h>
#include <sys/time.h>

#ifdef RT_USING_RTC

#define DBG_TAG "drv.rtc"
#define DBG_LVL DBG_INFO
#include <rtdbg.h>

#define BKUP_REG_DATA 0xA5A5

typedef struct
{
    struct rt_device device;
} rt_rtc_device;

static rt_rtc_device rtc_device;

static time_t get_rtc_timestamp(void)
{
    time_t rtc_counter;

    rtc_counter = (time_t)rtc_counter_get();

    return rtc_counter;
}

static rt_err_t set_rtc_timestamp(time_t time_stamp)
{
    uint32_t rtc_counter;

    rtc_counter = (uint32_t)time_stamp;

    rtc_lwoff_wait();
    rtc_configuration_mode_enter();
    rtc_counter_set(rtc_counter);
    rtc_configuration_mode_exit();
    rtc_lwoff_wait();

    return RT_EOK;
}

/**
 * @brief rtc时钟初始化
 */
static rt_err_t rt_rtc_init(void)
{
    rcu_periph_clock_enable(RCU_PMU);
    pmu_backup_write_enable();
    rcu_periph_clock_enable(RCU_BKPI);

    /* 在启动时检查备份寄存器BKP_DR1,如果内容不是 BKUP_REG_DATA,则需重新配置RTC */
    if (bkp_data_read(BKP_DATA_0) != BKUP_REG_DATA)
    {
#ifdef BSP_RTC_USING_LSI
        rcu_osci_on(RCU_IRC40K);                       /* 使能低速外部时钟 LSE */
        if (rcu_osci_stab_wait(RCU_IRC40K) != SUCCESS) /* 等待 LSE ready */
            return -RT_ERROR;

        rcu_rtc_clock_config(RCU_RTCSRC_IRC40K);
        rcu_periph_clock_enable(RCU_RTC);
        rtc_register_sync_wait();
        rtc_lwoff_wait();
        rtc_prescaler_set(40000 - 1);
        rtc_lwoff_wait();
#else
        rcu_osci_on(RCU_LXTAL);                       /* 使能低速外部时钟 LSE */
        if (rcu_osci_stab_wait(RCU_LXTAL) != SUCCESS) /* 等待 LSE ready */
            return -RT_ERROR;

        rcu_rtc_clock_config(RCU_RTCSRC_LXTAL); /* 选择LSE作为RTC时钟 */
        rcu_periph_clock_enable(RCU_RTC);       /* 使能RTC时钟 */
        rtc_register_sync_wait();               /* 等待RTC寄存器与APB1同步 */
        rtc_lwoff_wait();                       /* 等待最近一次对RTC寄存器的写操作完成 */
        rtc_prescaler_set(32767);
        rtc_lwoff_wait();
#endif
        bkp_data_write(BKP_DATA_0, BKUP_REG_DATA);
    }

    return RT_EOK;
}


static rt_err_t rt_rtc_control(rt_device_t dev, int cmd, void *args)
{
    rt_err_t result = RT_EOK;
    RT_ASSERT(dev != RT_NULL);

    switch (cmd)
    {
    case RT_DEVICE_CTRL_RTC_GET_TIME:
        *(rt_uint32_t *)args = get_rtc_timestamp();
        break;

    case RT_DEVICE_CTRL_RTC_SET_TIME:
        if (set_rtc_timestamp(*(rt_uint32_t *)args))
        {
            result = -RT_ERROR;
        }
        break;
    }

    return result;
}

#ifdef RT_USING_DEVICE_OPS
const static struct rt_device_ops rtc_ops =
    {
        RT_NULL,
        RT_NULL,
        RT_NULL,
        RT_NULL,
        RT_NULL,
        rt_rtc_control};
#endif

static int rt_hw_rtc_init(void)
{
    rt_err_t ret;
    
    if (rt_rtc_init() != RT_EOK)
        LOG_E("failed rt_rtc_init\r\n");

#ifdef RT_USING_DEVICE_OPS
    rtc_device.rtc_dev.ops = &rtc_ops;
#else
    rtc_device.device.init = RT_NULL;
    rtc_device.device.open = RT_NULL;
    rtc_device.device.close = RT_NULL;
    rtc_device.device.read = RT_NULL;
    rtc_device.device.write = RT_NULL;
    rtc_device.device.control = rt_rtc_control;
#endif
    rtc_device.device.type = RT_Device_Class_RTC;
    rtc_device.device.rx_indicate = RT_NULL;
    rtc_device.device.tx_complete = RT_NULL;
    rtc_device.device.user_data = RT_NULL;

    ret = rt_device_register(&rtc_device.device, "rtc", RT_DEVICE_FLAG_RDWR);
    if (ret != RT_EOK)
    {
        LOG_E("failed register internal rtc device, err=%d\r\n", ret);
    }

    return ret;
}
INIT_DEVICE_EXPORT(rt_hw_rtc_init);
#endif

/*********** (C) COPYRIGHT 2022 FiberHome *****END OF FILE****/
