//#include <rtthread.h>
//#include <rtdevice.h>
//#include "drv_spi.h"
//#include "drv_gpio.h"
//#define CS_PIN   GET_PIN(A,4)     //  SPI的CS引脚，PA4
//#define CLK_PIN  GET_PIN(A,5)    // SPI的CLK引脚，PA5
//#define MOSI_PIN  GET_PIN(A,7)    // SPI的MOSI引脚，PA7
//#define MISO_PIN  GET_PIN(A,6)    // SPI的MISO引脚，PA6

//#include <board.h>
//void rt_hw_us_delay(rt_uint32_t us)
//{
//    rt_uint32_t ticks;
//    rt_uint32_t told, tnow, tcnt = 0;
//    rt_uint32_t reload = SysTick->LOAD;

//    /* 获得延时经过的 tick 数 */
//    ticks = us * reload / (1000000 / RT_TICK_PER_SECOND);
//    /* 获得当前时间 */
//    told = SysTick->VAL;
//    while (1)
//    {
//        /* 循环获得当前时间，直到达到指定的时间后退出循环 */
//        tnow = SysTick->VAL;
//        if (tnow != told)
//        {
//            if (tnow < told)
//            {
//                tcnt += told - tnow;
//            }
//            else
//            {
//                tcnt += reload - tnow + told;
//            }
//            told = tnow;
//            if (tcnt >= ticks)
//            {
//                break;
//            }
//        }
//    }
//}

//void rt_spi_readsd_init_entry(void *parameter){
//		while(1)
//		{
//			rt_pin_write(CS_PIN, PIN_LOW); 
//			rt_hw_us_delay(1);    
//			int temp3 = 0XA6;
//			for(int i = 7;i >= 0;i--)
//			{
//				rt_pin_write(CLK_PIN, PIN_LOW);
//				rt_pin_write(MOSI_PIN, (temp3>>i)&0x1);
//				rt_hw_us_delay(1);
//				rt_pin_write(CLK_PIN, PIN_HIGH);
//				rt_hw_us_delay(1);
//			}		
//			int sd = 0x00;
//			for(int i = 7;i >= 0;i--)
//			{
//				rt_pin_write(CLK_PIN, PIN_LOW);
//				rt_hw_us_delay(1);
//				rt_pin_write(CLK_PIN, PIN_HIGH);
//				sd = sd|(rt_pin_read(MISO_PIN)<<i);
//				rt_hw_us_delay(1);
//			}
//			rt_pin_write(CS_PIN, PIN_HIGH); 
//			rt_kprintf("sensor data = 0x%x\n",sd);
//      rt_thread_mdelay(2000);
//		}
//}
///*编码器位置读取主要分为三步：
//	  激活寄存器(RACTIVE)和传感器(PACTIVE)
//		读取SDAD-status（看是否SV==1,检测传感器读取的位置数据是否有效）
//		SDAD_transmission读取传感器的位置数据
//*/
//static void spi_readSD(int argc, char *argv[]){
//		rt_thread_t tid_recv;
//			
//		//引脚初始化
//		rt_pin_mode(CS_PIN, PIN_MODE_OUTPUT);
//		rt_pin_mode(CLK_PIN, PIN_MODE_OUTPUT);
//		rt_pin_mode(MOSI_PIN, PIN_MODE_OUTPUT);
//		rt_pin_mode(MISO_PIN, PIN_MODE_INPUT);
//	
//		rt_pin_write(CS_PIN, PIN_HIGH);  
//		rt_pin_write(CLK_PIN, PIN_HIGH);
//	
//	//激活寄存器(RACTIVE)和传感器(PACTIVE) (OPCODE为0XB0)
//		rt_pin_write(CS_PIN, PIN_LOW); 
//		rt_thread_mdelay(1);
//		int temp1 = 0XB083;
//		for(int i = 15;i >= 0;i--)
//		{
//			rt_pin_write(CLK_PIN, PIN_LOW);
//			rt_pin_write(MOSI_PIN, (temp1>>i)&0x1);
//			rt_thread_mdelay(1);
//			rt_pin_write(CLK_PIN, PIN_HIGH);
//			rt_thread_mdelay(1);
//		}
//		rt_pin_write(CS_PIN, PIN_HIGH);  
//		rt_thread_mdelay(1);
//	//读取SDAD-status (OPCODE为0XF5)
//		rt_pin_write(CS_PIN, PIN_LOW); 
//		rt_thread_mdelay(1);
//		int temp2 = 0XF5;
//		for(int i = 7;i >= 0;i--)
//		{
//			rt_pin_write(CLK_PIN, PIN_LOW);
//			rt_pin_write(MOSI_PIN, (temp2>>i)&0x1);
//			rt_thread_mdelay(1);
//			rt_pin_write(CLK_PIN, PIN_HIGH);
//			rt_thread_mdelay(1);
//		}
//		rt_pin_write(CLK_PIN, PIN_LOW);
//		rt_thread_mdelay(1);
//		int sv = rt_pin_read(MISO_PIN);
//		rt_pin_write(CS_PIN, PIN_HIGH);  
//		rt_thread_mdelay(1);
//	//SDAD_transmission读取传感器的位置数据(OPCODE为0XA6)
//		if(sv)
//		{
//			rt_kprintf("enter read sensor data thread!\n");
//			tid_recv = rt_thread_create("co_cfg", rt_spi_readsd_init_entry, RT_NULL, 1024, 21, 10); 
//			if (tid_recv != RT_NULL)        
//				rt_thread_startup(tid_recv);
//		}
//}
////MSH_CMD_EXPORT(spi_readSD, spi read sensor data);
