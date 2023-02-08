#include <rtthread.h>
#include <rtdevice.h>
#include "drv_spi.h"
#include "drv_gpio.h"
#define MI_CS_PIN   GET_PIN(A,4)     //  SPI接收的165CS引脚，PA4, 
#define SPI_BUS_NAME                "spi0"
#define RECV_SPI_DEVICE_NAME   "spi01"
struct rt_spi_device *spi_dev_recv; 
rt_uint8_t send1_1 = 0xb0;
rt_uint8_t send1_2 = 0x83;
rt_uint8_t recv1_1 = 0x00;
rt_uint8_t recv1_2 = 0x00;
rt_uint8_t send2_1 = 0xf5;
rt_uint8_t send2_2 = 0x00;
rt_uint8_t recv2_1 = 0x00;
rt_uint8_t recv2_2 = 0x00;
rt_uint8_t send3_1 = 0xa6;
rt_uint8_t send3_2 = 0x00;
rt_uint8_t recv3_1 = 0x00;
rt_uint8_t recv3_2 = 0x00;

void rt_hw_spi_recv_init_entry(void *parameter){
	
		//激活寄存器(RACTIVE)和传感器(PACTIVE) (OPCODE为0XB0)
		struct rt_spi_message msg1, msg2;
		msg1.send_buf   = &send1_1;
		msg1.recv_buf   = &recv1_1;
		msg1.length     = 1;
		msg1.cs_take    = 1;
		msg1.cs_release = 0;
		msg1.next       = &msg2;
		msg2.send_buf   = &send1_2;
		msg2.recv_buf   = &recv1_2;
		msg2.length     = 1;
		msg2.cs_take    = 0;
		msg2.cs_release = 1;
		msg2.next       = RT_NULL;
		rt_spi_transfer_message(spi_dev_recv, &msg1);
		rt_kprintf("opcode1_1 is 0x%x\n",recv1_1);
		rt_kprintf("opcode1_2 is 0x%x\n",recv1_2);
		//读取SDAD-status (OPCODE为0XF5)
		msg1.send_buf   = &send2_1;
		msg1.recv_buf   = &recv2_1;
		msg1.length     = 1;
		msg1.cs_take    = 1;
		msg1.cs_release = 0;
		msg1.next       = &msg2;
		msg2.send_buf   = &send2_2;
		msg2.recv_buf   = &recv2_2;
		msg2.length     = 1;
		msg2.cs_take    = 0;
		msg2.cs_release = 1;
		msg2.next       = RT_NULL;
		rt_spi_transfer_message(spi_dev_recv, &msg1);
		rt_kprintf("opcode2_1 is 0x%x\n",recv2_1);
		rt_kprintf("opcode2_2 is 0x%x\n",recv2_2);		
		if(recv2_2 == 0x80)
		{
			while(1)
			{
				msg1.send_buf   = &send3_1;
				msg1.recv_buf   = &recv3_1;
				msg1.length     = 1;
				msg1.cs_take    = 1;
				msg1.cs_release = 0;
				msg1.next       = &msg2;
				msg2.send_buf   = &send3_2;
				msg2.recv_buf   = &recv3_2;
				msg2.length     = 1;
				msg2.cs_take    = 0;
				msg2.cs_release = 1;
				msg2.next       = RT_NULL;
				rt_spi_transfer_message(spi_dev_recv, &msg1);
				rt_kprintf("opcode3_1 is 0x%x\n",recv3_1);
				rt_kprintf("opcode3_2 is 0x%x\n",recv3_2);
				rt_thread_mdelay(2000);
			}
		}
	
//		rt_spi_send_then_send(spi_dev_recv,&send1_1,1,&send1_2,1);
//		rt_thread_mdelay(1);
//		rt_spi_send_then_recv(spi_dev_recv,&send2,1,&recv2,1);
//		rt_kprintf("sv is %d\n",recv2>>7);  
//		while(1)
//		{
//        rt_thread_mdelay(2000);
//				rt_spi_send_then_recv(spi_dev_recv,&send3,1,&recv3,1);
//				rt_kprintf("sd is 0x%x\n",recv3);
//		}
}
static void spi_recv_sample(int argc, char *argv[]){
		rt_thread_t tid_recv;
		rt_err_t res;
		char name[RT_NAME_MAX];
		if (argc == 2)
    {
        rt_strncpy(name, argv[1], RT_NAME_MAX);
    }
    else
    {
        rt_strncpy(name, RECV_SPI_DEVICE_NAME, RT_NAME_MAX);
    }
		res = rt_spi_device_attach(SPI_BUS_NAME,RECV_SPI_DEVICE_NAME, GPIOA, GPIO_PIN_4);             /* 绑定输入的CS片选引脚 */
		if(res != RT_EOK)
		{               
				rt_kprintf("%s attach failed\n",name); 
		}       
		else     
		{          
				rt_kprintf("%s attach sucess\n",name);  
		}       
		spi_dev_recv = (struct rt_spi_device *)rt_device_find(name);  /* 查找spi设备获取设备句柄 */   
		if (!spi_dev_recv)    
		{              
				rt_kprintf("spi sample run failed! can't find %s device!\n", name);
		}       
		else      
		{         
			/* config spi */       
				struct rt_spi_configuration cfg;             
				cfg.data_width = 8;           
				cfg.mode = RT_SPI_MASTER | RT_SPI_MODE_0 | RT_SPI_MSB; /* SPI Compatible: Mode 0 and Mode 3 */    
				cfg.max_hz = 1 * 1000 * 1000; /* 1M */      
				rt_spi_configure(spi_dev_recv, &cfg);    
		}     
		tid_recv = rt_thread_create("co_cfg", rt_hw_spi_recv_init_entry, RT_NULL, 1024, 21, 10); 
		if (tid_recv != RT_NULL)        
			rt_thread_startup(tid_recv);
}
MSH_CMD_EXPORT(spi_recv_sample, spi recv sample);
