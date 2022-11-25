/**
 * @file drv_flash.c
 * @brief 
 * @author wangh (wanghuan3037@fiberhome.com)
 * @version 1.0
 * @date 2021-12-20
 * @copyright Copyright (c) 2021  烽火通信
 */ 
#include "board.h"

#ifdef BSP_USING_ON_CHIP_FLASH
#include "drv_flash.h"

#if defined(PKG_USING_FAL)
#include "fal.h"
#endif

#define DBG_LEVEL           DBG_INFO
#define LOG_TAG             "drv.flash"
#include <rtdbg.h>

/**
 * @brief 获取指定地址的flash页边界
 * @param[in]  addr     地址
 * @return uint32_t 页边界地址
 */
static uint32_t GetPage(uint32_t addr)
{
    uint32_t page = 0;
    page = RT_ALIGN_DOWN(addr, GD32_FLASH_PAGE_SIZE);
    return page;
}


/**
 * @brief Read data from flash.
 * @note This operation's units is word.
 * @param[in]  addr     flash address
 * @param[out] buf      buffer to store read data
 * @param[in]  size     read bytes size
 * @return int 
 */
int gd32_flash_read(rt_uint32_t addr, rt_uint8_t *buf, size_t size)
{
    size_t i;

    if ((addr + size) > GD32_FLASH_END_ADDRESS)
    {
        LOG_E("read outrange flash size! addr is (0x%p)\r\n", (void *)(addr + size));
        return -RT_EINVAL;
    }

    for (i = 0; i < size; i++, buf++, addr++)
    {
        *buf = *(rt_uint8_t *) addr;
    }

    return size;
}

/**
 * Write data to flash.
 * @note This operation's units is word.
 * @note This operation must after erase. @see flash_erase.
 *
 * @param addr flash address
 * @param buf the write data buffer
 * @param size write bytes size
 *
 * @return result
 */
int gd32_flash_write(rt_uint32_t addr, const rt_uint8_t *buf, size_t size)
{
    rt_err_t result        = RT_EOK;
    rt_uint32_t end_addr   = addr + size;

    if (addr % 4 != 0)
    {
        LOG_E("write addr must be 4-byte alignment\r\n");
        return -RT_EINVAL;
    }

    if ((end_addr) > GD32_FLASH_END_ADDRESS)
    {
        LOG_E("write outrange flash size! addr is (0x%p)\r\n", (void *)(addr + size));
        return -RT_EINVAL;
    }

    /* unlock the internal flash */
    fmc_unlock();

    while (addr < end_addr)
    {
        if (fmc_word_program(addr, *((rt_uint32_t *)buf)) == FMC_READY)
        {
            if (*(rt_uint32_t *)addr != *(rt_uint32_t *)buf)
            {
                result = -RT_ERROR;
                break;
            }
            addr += 4;
            buf  += 4;
        }
        else
        {
            result = -RT_ERROR;
            break;
        }
    }

    fmc_lock();

    if (result != RT_EOK)
    {
        return result;
    }

    return size;
}

/**
 * @brief flash 擦除 \n
 * 会计算起始地址到所需擦除长度占用的所有扇区
 * @param[in]  addr     起始地址（4字节边界)
 * @param[in]  size     擦除长度（4字节倍数）
 * @return int 0:成功，-1：失败
 */
int gd32_flash_erase(rt_uint32_t addr, size_t size)
{
    rt_err_t result = RT_EOK;
    uint32_t page_addr;


    if ((addr + size) > GD32_FLASH_END_ADDRESS)
    {
        LOG_E("ERROR: erase outrange flash size! addr is (0x%p)\n", (void *)(addr + size));
        return -RT_EINVAL;
    }

    fmc_unlock();
    page_addr = GetPage(addr);

    while ( page_addr < (addr + size) )
    {
        if ( page_addr < FMC_BANK0_END_ADDRESS )
        {
            fmc_flag_clear(FMC_FLAG_BANK0_END);
            fmc_flag_clear(FMC_FLAG_BANK0_WPERR);
            fmc_flag_clear(FMC_FLAG_BANK0_PGERR);
        }
        else
        {
            fmc_flag_clear(FMC_FLAG_BANK1_END);
            fmc_flag_clear(FMC_FLAG_BANK1_WPERR);
            fmc_flag_clear(FMC_FLAG_BANK1_PGERR);
        }

        if ( FMC_READY != fmc_page_erase(page_addr) )
        {
            result = -RT_ERROR;
            goto __exit;
        }

        page_addr += GD32_FLASH_PAGE_SIZE;
    }

__exit:
    fmc_lock();

    if (result != RT_EOK)
    {
        return result;
    }

    LOG_D("erase done: addr (0x%p), size %d\r\n", (void *)addr, size);
    return size;
}



#if defined(PKG_USING_FAL)

static int fal_flash_read(long offset, rt_uint8_t *buf, size_t size);
static int fal_flash_write(long offset, const rt_uint8_t *buf, size_t size);
static int fal_flash_erase(long offset, size_t size);

const struct fal_flash_dev gd32_onchip_flash = { "onchip_flash", GD32_FLASH_START_ADRESS, GD32_FLASH_SIZE, GD32_FLASH_PAGE_SIZE, {NULL, fal_flash_read, fal_flash_write, fal_flash_erase} };

static int fal_flash_read(long offset, rt_uint8_t *buf, size_t size)
{
    return gd32_flash_read(gd32_onchip_flash.addr + offset, buf, size);
}

static int fal_flash_write(long offset, const rt_uint8_t *buf, size_t size)
{
    return gd32_flash_write(gd32_onchip_flash.addr + offset, buf, size);
}

static int fal_flash_erase(long offset, size_t size)
{
    return gd32_flash_erase(gd32_onchip_flash.addr + offset, size);
}

#endif
#endif /* BSP_USING_ON_CHIP_FLASH */


/*********** (C) COPYRIGHT 2021 FiberHome *****END OF FILE****/
