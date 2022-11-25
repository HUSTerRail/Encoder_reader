#include <rtthread.h>
#include <rtdevice.h>
#include "drv_spi.h"
#include "drv_gpio.h"
#define MI_CS_PIN   GET_PIN(A,4)     //  SPI接收的165CS引脚，PA4, 
#define SPI_BUS_NAME                "spi0"
#define RECV_SPI_DEVICE_NAME   "spi01"
struct rt_spi_device *spi_dev_recv; 
rt_uint8_t recv[3] = {0};
rt_uint8_t send[3] = {0};

void rt_hw_spi_recv_init_entry(void *parameter){
		while(1)
		{
        rt_thread_mdelay(2000);
        rt_spi_recv(spi_dev_recv,&recv,3);   //接收数据
        rt_kprintf("spi_recv is 0x%x、0x%x、0x%x\n",recv[0], recv[1],recv[2]);   
				rt_spi_send(spi_dev_recv,send,3);    //发送数据
				
		}
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
