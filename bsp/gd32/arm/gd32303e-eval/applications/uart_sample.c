/*
 * 程序清单：这是一个 串口 设备使用例程
 * 例程导出了 uart_sample 命令到控制终端
 * 命令调用格式：uart_sample uart2
 * 命令解释：命令第二个参数是要使用的串口设备名称，为空则使用默认的串口设备
 * 程序功能：通过串口输出字符串"hello RT-Thread!"，然后错位输出输入的字符
*/

#include <rtthread.h>
#include <rtdevice.h>
#include "definitions.h"
#define SAMPLE_UART_NAME       "uart2"


/* 用于接收消息的信号量 */
struct rt_semaphore rx_sem;
struct rt_semaphore tx_sem;
struct rt_semaphore spi_sem;

extern int Factory_Baud_Rate;
extern int Factory_Position_Offset;

static rt_device_t serial;
int size_count = 0; //编程命令执行后，后续需要跟随的命令字节数量
rt_uint32_t baud_rate = 0;  //波特率
rt_uint32_t position_offset = 0;  //位置偏移量
rt_uint8_t CMD = 0;  //调用指令
rt_uint8_t Position_EW[3] = {0,0,0};  //3bytes (Position + E/W bits)

/* 接收数据回调函数 */
static rt_err_t uart_input(rt_device_t dev, rt_size_t size)
{
    /* 串口接收到数据后产生中断，调用此回调函数，然后发送接收信号量 */
    rt_sem_release(&rx_sem);

    return RT_EOK;
}

//Addr. SER对应串口访问寄存器的ROM地址，ADDR代表串口对应MU150的RAM地址
static void serial_thread_entry(void *parameter)
{
    unsigned char ch;
		static int count = 0;
		unsigned char obay[5] = {0,0xCD,0XEF,0X89,0XAB};
		extern rt_uint8_t Read_Register(rt_uint8_t ADDR);
		extern void Write_Register(rt_uint8_t ADDR,rt_uint8_t data);
		
    while (1)
    {
        /* 从串口读取一个字节的数据，没有读取到则等待接收信号量 */
        while (rt_device_read(serial, -1, &ch, 1) != 1)
        {
            /* 阻塞等待接收信号量，等到信号量后再次读取数据 */
            rt_sem_take(&rx_sem, RT_WAITING_FOREVER);
        }
        /* 读取到的数据通过串口错位输出 */
				if(CMD == 0X5A && size_count > 0){   //位置偏移量的4个字节获取
					size_count--;
					position_offset = (position_offset<<8) + ch;
					if(size_count == 0){
						if(position_offset > 524287)
							position_offset = Factory_Position_Offset;
					}
				}
				else if(CMD == 0x42 && size_count > 0){   //波特率的4个字节获取
					size_count--;
					baud_rate = (baud_rate<<8) + ch;
					if(size_count == 0){
						if(baud_rate != 3000000 || baud_rate != 2000000 || baud_rate != 921600 || baud_rate != 460800 || baud_rate != 230400
							|| baud_rate != 115200 || baud_rate != 57600 || baud_rate != 38400 || baud_rate != 19200
							|| baud_rate != 9600 || baud_rate != 4800 || baud_rate != 2400)
							baud_rate = Factory_Baud_Rate;
						//先关闭串口再打开，重新设置波特率
						char uart_name[RT_NAME_MAX];
						rt_strncpy(uart_name, SAMPLE_UART_NAME, RT_NAME_MAX);
						serial = rt_device_find(uart_name);
						rt_device_close(serial); 
						struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT; 
						config.baud_rate = baud_rate; 
						rt_device_control(serial, RT_DEVICE_CTRL_CONFIG, &config);
						rt_device_open(serial, RT_DEVICE_FLAG_INT_RX);
						rt_device_set_rx_indicate(serial, uart_input);						
					}
				}
				else if(count == 4){    //是否完成编码器的解锁（unlock）
					count = 0;
					if(ch == 0X5A)  //位置偏移值设置(后续需要增加4个字节)
					{
						CMD = 0X5A;
						size_count = 4;
						rt_device_write(serial, 0, &ch, 1);
					}
					else if(ch == 0x63) //配置参数保存
					{
						CMD = 0x63;
						rt_device_write(serial, 0, &ch, 1);
						rt_sem_release(&spi_sem);
						rt_sem_take(&tx_sem, RT_WAITING_FOREVER);
					}
					else if(ch == 0x72) //重置为出厂设置
					{
						CMD = 0x72;
						rt_device_write(serial, 0, &ch, 1);
						position_offset = Factory_Position_Offset;
						baud_rate = Factory_Baud_Rate;
						rt_sem_release(&spi_sem);
						rt_sem_take(&tx_sem, RT_WAITING_FOREVER);
						//先关闭串口再打开，重新设置波特率
						char uart_name[RT_NAME_MAX];
						rt_strncpy(uart_name, SAMPLE_UART_NAME, RT_NAME_MAX);
						serial = rt_device_find(uart_name);
						rt_device_close(serial); 
						struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT; 
						config.baud_rate = baud_rate; 
						rt_device_control(serial, RT_DEVICE_CTRL_CONFIG, &config);
						rt_device_open(serial, RT_DEVICE_FLAG_INT_RX);
						rt_device_set_rx_indicate(serial, uart_input);							
					}
					else if(ch == 0x42)  //波特率设置(后续需要增加4个字节)
					{
						CMD = 0x42;
						size_count = 4;
						rt_device_write(serial, 0, &ch, 1);
					}
				}
				else if((ch == 0xCD || ch == 0XEF || ch == 0X89 || ch == 0XAB) && count < 4){
					rt_device_write(serial, 0, &ch, 1);  //立刻返回初值
					count++;
					if(ch == obay[count])
						;
					else{
						count = 0;
					}
				}
				else if(ch == 0x31 || ch == 0X33){
					CMD = ch;
					count = 0;
					if(ch == 0X31)
						rt_device_write(serial, 0, &ch, 1);	
					rt_sem_release(&spi_sem);
					rt_sem_take(&tx_sem, RT_WAITING_FOREVER);
					rt_device_write(serial, 0, &Position_EW, 3);		
					if((Position_EW[2] & 0x3) == 0x01)  //表示出现了问题
						error_handling();
				}
    }
}

int uart_sample(int buad_rate)
{
    rt_err_t ret = RT_EOK;
    char uart_name[RT_NAME_MAX];
//    char str[] = "hello RT-Thread!\r\n";

    rt_strncpy(uart_name, SAMPLE_UART_NAME, RT_NAME_MAX);

    /* 查找系统中的串口设备 */
    serial = rt_device_find(uart_name);
    if (!serial)
    {
        rt_kprintf("find %s failed!\n", uart_name);
        return RT_ERROR;
    }
		struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;  /* 初始化配置参数 */
		config.baud_rate = buad_rate; 
		/* step3：控制串口设备。通过控制接口传入命令控制字，与控制参数 */
		rt_device_control(serial, RT_DEVICE_CTRL_CONFIG, &config);
		
    /* 以中断接收及轮询发送模式打开串口设备 */
    rt_device_open(serial, RT_DEVICE_FLAG_INT_RX);
    /* 设置接收回调函数 */
    rt_device_set_rx_indicate(serial, uart_input);
    /* 发送字符串 */
//    rt_device_write(serial, 0, str, (sizeof(str) - 1));

    /* 创建 serial 线程 */
    rt_thread_t thread = rt_thread_create("serial", serial_thread_entry, RT_NULL, 1024, 25, 10);
    /* 创建成功则启动线程 */
    if (thread != RT_NULL)
    {
        rt_thread_startup(thread);
    }
    else
    {
        ret = RT_ERROR;
    }

    return ret;
}
//INIT_APP_EXPORT(uart_sample);
