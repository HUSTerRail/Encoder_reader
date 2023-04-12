/*
 * �����嵥������һ�� ���� �豸ʹ������
 * ���̵����� uart_sample ��������ն�
 * ������ø�ʽ��uart_sample uart2
 * ������ͣ�����ڶ���������Ҫʹ�õĴ����豸���ƣ�Ϊ����ʹ��Ĭ�ϵĴ����豸
 * �����ܣ�ͨ����������ַ���"hello RT-Thread!"��Ȼ���λ���������ַ�
*/

#include <rtthread.h>
#include <rtdevice.h>
#include "definitions.h"
#define SAMPLE_UART_NAME       "uart2"


/* ���ڽ�����Ϣ���ź��� */
struct rt_semaphore rx_sem;
struct rt_semaphore tx_sem;
struct rt_semaphore spi_sem;

extern int Factory_Baud_Rate;
extern int Factory_Position_Offset;

static rt_device_t serial;
int size_count = 0; //�������ִ�к󣬺�����Ҫ����������ֽ�����
rt_uint32_t baud_rate = 0;  //������
rt_uint32_t position_offset = 0;  //λ��ƫ����
rt_uint8_t CMD = 0;  //����ָ��
rt_uint8_t Position_EW[3] = {0,0,0};  //3bytes (Position + E/W bits)

/* �������ݻص����� */
static rt_err_t uart_input(rt_device_t dev, rt_size_t size)
{
    /* ���ڽ��յ����ݺ�����жϣ����ô˻ص�������Ȼ���ͽ����ź��� */
    rt_sem_release(&rx_sem);

    return RT_EOK;
}

//Addr. SER��Ӧ���ڷ��ʼĴ�����ROM��ַ��ADDR�����ڶ�ӦMU150��RAM��ַ
static void serial_thread_entry(void *parameter)
{
    unsigned char ch;
		static int count = 0;
		unsigned char obay[5] = {0,0xCD,0XEF,0X89,0XAB};
		extern rt_uint8_t Read_Register(rt_uint8_t ADDR);
		extern void Write_Register(rt_uint8_t ADDR,rt_uint8_t data);
		
    while (1)
    {
        /* �Ӵ��ڶ�ȡһ���ֽڵ����ݣ�û�ж�ȡ����ȴ������ź��� */
        while (rt_device_read(serial, -1, &ch, 1) != 1)
        {
            /* �����ȴ������ź������ȵ��ź������ٴζ�ȡ���� */
            rt_sem_take(&rx_sem, RT_WAITING_FOREVER);
        }
        /* ��ȡ��������ͨ�����ڴ�λ��� */
				if(CMD == 0X5A && size_count > 0){   //λ��ƫ������4���ֽڻ�ȡ
					size_count--;
					position_offset = (position_offset<<8) + ch;
					if(size_count == 0){
						if(position_offset > 524287)
							position_offset = Factory_Position_Offset;
					}
				}
				else if(CMD == 0x42 && size_count > 0){   //�����ʵ�4���ֽڻ�ȡ
					size_count--;
					baud_rate = (baud_rate<<8) + ch;
					if(size_count == 0){
						if(baud_rate != 3000000 || baud_rate != 2000000 || baud_rate != 921600 || baud_rate != 460800 || baud_rate != 230400
							|| baud_rate != 115200 || baud_rate != 57600 || baud_rate != 38400 || baud_rate != 19200
							|| baud_rate != 9600 || baud_rate != 4800 || baud_rate != 2400)
							baud_rate = Factory_Baud_Rate;
						//�ȹرմ����ٴ򿪣��������ò�����
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
				else if(count == 4){    //�Ƿ���ɱ������Ľ�����unlock��
					count = 0;
					if(ch == 0X5A)  //λ��ƫ��ֵ����(������Ҫ����4���ֽ�)
					{
						CMD = 0X5A;
						size_count = 4;
						rt_device_write(serial, 0, &ch, 1);
					}
					else if(ch == 0x63) //���ò�������
					{
						CMD = 0x63;
						rt_device_write(serial, 0, &ch, 1);
						rt_sem_release(&spi_sem);
						rt_sem_take(&tx_sem, RT_WAITING_FOREVER);
					}
					else if(ch == 0x72) //����Ϊ��������
					{
						CMD = 0x72;
						rt_device_write(serial, 0, &ch, 1);
						position_offset = Factory_Position_Offset;
						baud_rate = Factory_Baud_Rate;
						rt_sem_release(&spi_sem);
						rt_sem_take(&tx_sem, RT_WAITING_FOREVER);
						//�ȹرմ����ٴ򿪣��������ò�����
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
					else if(ch == 0x42)  //����������(������Ҫ����4���ֽ�)
					{
						CMD = 0x42;
						size_count = 4;
						rt_device_write(serial, 0, &ch, 1);
					}
				}
				else if((ch == 0xCD || ch == 0XEF || ch == 0X89 || ch == 0XAB) && count < 4){
					rt_device_write(serial, 0, &ch, 1);  //���̷��س�ֵ
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
					if((Position_EW[2] & 0x3) == 0x01)  //��ʾ����������
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

    /* ����ϵͳ�еĴ����豸 */
    serial = rt_device_find(uart_name);
    if (!serial)
    {
        rt_kprintf("find %s failed!\n", uart_name);
        return RT_ERROR;
    }
		struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;  /* ��ʼ�����ò��� */
		config.baud_rate = buad_rate; 
		/* step3�����ƴ����豸��ͨ�����ƽӿڴ�����������֣�����Ʋ��� */
		rt_device_control(serial, RT_DEVICE_CTRL_CONFIG, &config);
		
    /* ���жϽ��ռ���ѯ����ģʽ�򿪴����豸 */
    rt_device_open(serial, RT_DEVICE_FLAG_INT_RX);
    /* ���ý��ջص����� */
    rt_device_set_rx_indicate(serial, uart_input);
    /* �����ַ��� */
//    rt_device_write(serial, 0, str, (sizeof(str) - 1));

    /* ���� serial �߳� */
    rt_thread_t thread = rt_thread_create("serial", serial_thread_entry, RT_NULL, 1024, 25, 10);
    /* �����ɹ��������߳� */
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
