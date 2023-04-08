#include <rtthread.h>
#include <rtdevice.h>
#include "drv_spi.h"
#include "drv_gpio.h"
#include "definitions.h"
#define MI_CS_PIN   GET_PIN(A,4)     //  SPI接收的165CS引脚，PA4, 
#define LED_ERR GET_PIN(B, 3)  //ERR灯
#define SPI_BUS_NAME                "spi0"
#define RECV_SPI_DEVICE_NAME   "spi01"
struct rt_spi_device *spi_dev_recv; 

int Factory_Baud_Rate = 115200;   //出厂波特率设置为115200
int Factory_Position_Offset = 0;   //出厂位置偏移值设置为0，新位置=绝对位置-偏移量
extern struct rt_semaphore rx_sem;
extern struct rt_semaphore tx_sem;
extern struct rt_semaphore spi_sem;

extern rt_uint8_t CMD;
extern rt_uint32_t baud_rate;
extern rt_uint32_t position_offset;
extern rt_uint8_t Position_EW[3];

rt_uint8_t send1_1 = 0xb0;  //ACTIVATE
rt_uint8_t send1_2 = 0x83;
rt_uint8_t recv1_1 = 0x00;
rt_uint8_t recv1_2 = 0x00;
rt_uint8_t send2_1 = 0xf5;  //SDAD Status (no latch)
rt_uint8_t send2_2 = 0x00;
rt_uint8_t recv2_1 = 0x00;
rt_uint8_t recv2_2 = 0x00;
rt_uint8_t send3_1 = 0xa6;  //SDAD-transmission (sensor data)
rt_uint8_t send3_2 = 0x00;
rt_uint8_t send3_3 = 0x00;
rt_uint8_t send3_4 = 0x00;
rt_uint8_t recv3_1 = 0x00;
rt_uint8_t recv3_2 = 0x00;
rt_uint8_t recv3_3 = 0x00;
rt_uint8_t recv3_4 = 0x00;
rt_uint8_t send4_1 = 0xAD;  //REGISTER status/data
rt_uint8_t send4_2 = 0x00;
rt_uint8_t recv4_1 = 0x00;
rt_uint8_t recv4_2 = 0xFF;

void error_handling(void){   //寄存器或传感器数据的读取出现了问题
	while(1){
		rt_pin_write(LED_ERR, PIN_LOW);
	}
}

void rt_hw_spi_recv_init_entry(void *parameter){
		struct rt_spi_message msg1,msg2,msg3,msg4;
		//激活寄存器(RACTIVE)和传感器(PACTIVE) (OPCODE为0XB0)
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
//		rt_kprintf("opcode1_1 is 0x%x\n",recv1_1);
//		rt_kprintf("opcode1_2 is 0x%x\n",recv1_2);		
		//刚开机时需从EEPROM读取位置偏移量（设置寄存器地址为：0x19(b31~b24),0x1A（b23 ~b16）,0x1B（b15 ~b8）,0X1C（b7 ~b0））
	  //读寄存器
		rt_uint8_t a1 = 0x97,a2 = 0x19,a3 = 0xad,a4 = 0x00,a5 = 0x00;
		rt_uint8_t b1 = 0x00,b2 = 0x00,b3 = 0x00,b4 = 0xFF,b5 = 0x00;  //b4对应status,b5对应寄存器data
		for(int i = 0;i < 4;i++){
			while(b4 & 0x04)  //FAIL，读取数据失败则一直读取（对应STATUS第3位）
			{
				msg1.send_buf   = &a1;
				msg1.recv_buf   = &b1;
				msg1.length     = 1;
				msg1.cs_take    = 1;
				msg1.cs_release = 0;
				msg1.next       = &msg2;
				msg2.send_buf   = &a2;
				msg2.recv_buf   = &b2;
				msg2.length     = 1;
				msg2.cs_take    = 0;
				msg2.cs_release = 1;
				msg2.next       = RT_NULL;
				rt_spi_transfer_message(spi_dev_recv, &msg1);
				while(b4 & 0x2){  //busy标志位判断
					msg1.send_buf   = &a3;
					msg1.recv_buf   = &b3;
					msg1.length     = 1;
					msg1.cs_take    = 1;
					msg1.cs_release = 0;
					msg1.next       = &msg2;
					msg2.send_buf   = &a4;
					msg2.recv_buf   = &b4;
					msg2.length     = 1;
					msg2.cs_take    = 1;
					msg2.cs_release = 0;
					msg2.next       = &msg3;
					msg3.send_buf   = &a5;
					msg3.recv_buf   = &b5;
					msg3.length     = 1;
					msg3.cs_take    = 0;
					msg3.cs_release = 1;
					msg3.next       = RT_NULL;
					rt_spi_transfer_message(spi_dev_recv, &msg1);	
				}
			if(b4 & 0x1) //数据是否有效
				break;
		}
			if(b4 & 0x1) //数据是否有效
			{
				position_offset = (position_offset<<8) + b5;
			}
			else if(b4 & 0x88)  //DISMISS,ERROR，发生错误
				error_handling();
			a2++;
		}
		//刚开机时需从EEPROM读取波特率(设置寄存器地址为：0x26(b31~b24),0x27（b23 ~b16）,0x28（b15 ~b8）,0X29（b7 ~b0）)
		b4 = 0xff;
		a2 = 0x26;
		for(int i = 0;i < 4;i++){
			while(b4 & 0x04)  //FAIL，读取数据失败则一直读取（对应STATUS第3位）
			{
				//读寄存器
				msg1.send_buf   = &a1;
				msg1.recv_buf   = &b1;
				msg1.length     = 1;
				msg1.cs_take    = 1;
				msg1.cs_release = 0;
				msg1.next       = &msg2;
				msg2.send_buf   = &a2;
				msg2.recv_buf   = &b2;
				msg2.length     = 1;
				msg2.cs_take    = 0;
				msg2.cs_release = 1;
				msg2.next       = RT_NULL;
				rt_spi_transfer_message(spi_dev_recv, &msg1);
				while(b4 & 0x2){  //busy标志位判断
					//读状态位/数据位
					msg1.send_buf   = &a3;
					msg1.recv_buf   = &b3;
					msg1.length     = 1;
					msg1.cs_take    = 1;
					msg1.cs_release = 0;
					msg1.next       = &msg2;
					msg2.send_buf   = &a4;
					msg2.recv_buf   = &b4;
					msg2.length     = 1;
					msg2.cs_take    = 1;
					msg2.cs_release = 0;
					msg2.next       = &msg3;
					msg3.send_buf   = &a5;
					msg3.recv_buf   = &b5;
					msg3.length     = 1;
					msg3.cs_take    = 0;
					msg3.cs_release = 1;
					msg3.next       = RT_NULL;
					rt_spi_transfer_message(spi_dev_recv, &msg1);	
				}
				if(b4 & 0x1) //数据是否有效
					break;
			}
			if(b4 & 0x1) //数据是否有效
			{
				baud_rate = (baud_rate<<8) + b5;
			}
			else if(b4 & 0x88)  //DISMISS,ERROR,发生错误
				error_handling();
			a2++;
		}		
		if(baud_rate == 0 || baud_rate != 3000000 || baud_rate != 2000000 || baud_rate != 921600 || baud_rate != 460800
			 || baud_rate != 230400 || baud_rate != 115200 || baud_rate != 57600 || baud_rate != 38400 || baud_rate != 19200
			 || baud_rate != 9600 || baud_rate != 4800 || baud_rate != 2400){
			baud_rate = Factory_Baud_Rate; //设置为默认出厂波特率
		}
		uart_sample(baud_rate); //串口初始化函数
		rt_thread_mdelay(5);
			
		while(1){
			rt_sem_take(&spi_sem, RT_WAITING_FOREVER);
			if(CMD == 0X63 || CMD == 0X72)  //配置参数保存或重置为出厂设置
			{
				//写寄存器功能
				rt_uint8_t params_set_send1 = 0xD2, params_set_send2 = 0x19,params_set_send3 = 0x00,
									 params_set_send4 = 0xad,params_set_send5 = 0x00,params_set_send6 = 0x00;
				rt_uint8_t params_set_recv1 = 0x00,params_set_recv2 = 0x00,params_set_recv3 = 0x00,
									 params_set_recv4 = 0xad,params_set_recv5 = 0xFF,params_set_recv6 = 0x00;
				rt_uint8_t position[4] = {0,0,0,0};
				position[0] = (position_offset & 0xff000000)>>24;
				position[1] = (position_offset & 0x00ff0000)>>16;
				position[2] = (position_offset & 0x0000ff00)>>8;
				position[3] = (position_offset & 0x000000ff);
				for(int i = 0;i < 4;i++){  //先写位置偏移值
					params_set_send3 = position[i];
					while(params_set_recv5 & 0x04) //FAIL，读取数据失败则一直读取（对应STATUS第3位）
					{
						//写寄存器
						msg1.send_buf   = &params_set_send1;
						msg1.recv_buf   = &params_set_recv1;
						msg1.length     = 1;
						msg1.cs_take    = 1;
						msg1.cs_release = 0;
						msg1.next       = &msg2;
						msg2.send_buf   = &params_set_send2;
						msg2.recv_buf   = &params_set_recv2;
						msg2.length     = 1;
						msg2.cs_take    = 1;
						msg2.cs_release = 0;
						msg2.next       = &msg3;
						msg3.send_buf   = &params_set_send3;
						msg3.recv_buf   = &params_set_recv3;
						msg3.length     = 1;
						msg3.cs_take    = 0;
						msg3.cs_release = 1; 
						msg3.next       = RT_NULL;
						rt_spi_transfer_message(spi_dev_recv, &msg1);	
					while(params_set_recv5 & 0x2){  //busy标志位判断
						//读状态位/数据位
						msg1.send_buf   = &params_set_send4;
						msg1.recv_buf   = &params_set_recv4;
						msg1.length     = 1;
						msg1.cs_take    = 1;
						msg1.cs_release = 0;
						msg1.next       = &msg2;
						msg2.send_buf   = &params_set_send5;
						msg2.recv_buf   = &params_set_recv5;
						msg2.length     = 1;
						msg2.cs_take    = 1;
						msg2.cs_release = 0;
						msg2.next       = &msg3;
						msg3.send_buf   = &params_set_send6;
						msg3.recv_buf   = &params_set_recv6;
						msg3.length     = 1;
						msg3.cs_take    = 0;
						msg3.cs_release = 1;
						msg3.next       = RT_NULL;
						rt_spi_transfer_message(spi_dev_recv, &msg1);	
					}
					if(params_set_recv5 & 0x1) //数据是否有效
						break;					
					}
				if(params_set_recv5 & 0x88)  //DISMISS,ERROR，发生错误
					error_handling();
				params_set_send2++;					
			}
				rt_uint8_t baud[4] = {0,0,0,0};
				baud[0] = (baud_rate & 0xff000000)>>24;
				baud[1] = (baud_rate & 0x00ff0000)>>16;
				baud[2] = (baud_rate & 0x0000ff00)>>8;
				baud[3] = (baud_rate & 0x000000ff);
				params_set_recv5 = 0xff;
				params_set_send2 = 0x26;
				for(int i = 0;i < 4;i++){  //先写位置偏移值
					params_set_send3 = baud[i];
					while(params_set_recv5 & 0x04) //FAIL，读取数据失败则一直读取（对应STATUS第3位）
					{
						//写寄存器
						msg1.send_buf   = &params_set_send1;
						msg1.recv_buf   = &params_set_recv1;
						msg1.length     = 1;
						msg1.cs_take    = 1;
						msg1.cs_release = 0;
						msg1.next       = &msg2;
						msg2.send_buf   = &params_set_send2;
						msg2.recv_buf   = &params_set_recv2;
						msg2.length     = 1;
						msg2.cs_take    = 1;
						msg2.cs_release = 0;
						msg2.next       = &msg3;
						msg3.send_buf   = &params_set_send3;
						msg3.recv_buf   = &params_set_recv3;
						msg3.length     = 1;
						msg3.cs_take    = 0;
						msg3.cs_release = 1; 
						msg3.next       = RT_NULL;
						rt_spi_transfer_message(spi_dev_recv, &msg1);	
					while(params_set_recv5 & 0x2){  //busy标志位判断
						//读状态位/数据位
						msg1.send_buf   = &params_set_send4;
						msg1.recv_buf   = &params_set_recv4;
						msg1.length     = 1;
						msg1.cs_take    = 1;
						msg1.cs_release = 0;
						msg1.next       = &msg2;
						msg2.send_buf   = &params_set_send5;
						msg2.recv_buf   = &params_set_recv5;
						msg2.length     = 1;
						msg2.cs_take    = 1;
						msg2.cs_release = 0;
						msg2.next       = &msg3;
						msg3.send_buf   = &params_set_send6;
						msg3.recv_buf   = &params_set_recv6;
						msg3.length     = 1;
						msg3.cs_take    = 0;
						msg3.cs_release = 1;
						msg3.next       = RT_NULL;
						rt_spi_transfer_message(spi_dev_recv, &msg1);	
					}
					if(params_set_recv5 & 0x1) //数据是否有效
						break;					
					}
				if(params_set_recv5 & 0x88)  //DISMISS,ERROR，发生错误
					error_handling();
				params_set_send2++;					
			}			
			}
			else if(CMD == 0X31 || CMD == 0X33)  //返回3bytes (Position + E/W bits)
			{
				//读取SDAD-status (OPCODE为0XF5)
				recv2_2 = 0x00;
				while(recv2_2 != 0x80)  //如果传感器一直无效（SV == 0）,则一直读取SDAD-status
				{
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
				}
				//读取SD-data
				while(recv4_2 & 0x04)  //FAIL，读取数据失败则一直读取（对应STATUS第3位）
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
					msg2.cs_take    = 1;
					msg2.cs_release = 0;
					msg2.next       = &msg3;
					msg3.send_buf   = &send3_3;
					msg3.recv_buf   = &recv3_3;
					msg3.length     = 1;
					msg3.cs_take    = 1;
					msg3.cs_release = 0;
					msg3.next       = &msg4;		
					msg4.send_buf   = &send3_4;
					msg4.recv_buf   = &recv3_4;
					msg4.length     = 1;
					msg4.cs_take    = 0;
					msg4.cs_release = 1;
					msg4.next       = NULL;					
					rt_spi_transfer_message(spi_dev_recv, &msg1);
					//REGISTER status/data
					while(recv4_2 & 0x02)   //BUSY，读取数据繁忙则一直刷新状态（对应STATUS第2位）
					{
						msg1.send_buf   = &send4_1;
						msg1.recv_buf   = &recv4_1;
						msg1.length     = 1;
						msg1.cs_take    = 1;
						msg1.cs_release = 0;
						msg1.next       = &msg2;
						msg2.send_buf   = &send4_2;
						msg2.recv_buf   = &recv4_2;
						msg2.length     = 1;
						msg2.cs_take    = 0;
						msg2.cs_release = 1;
						msg2.next       = RT_NULL;
						rt_spi_transfer_message(spi_dev_recv, &msg1);
					}
					if((recv4_2 & 0x01) == 1) //VALID,代表读取数据有效
					{
						break;  //跳出当前的循环
					}
				}
				//需要将得到的绝对位置减去位置偏移值，即得到当前的位置
				rt_int32_t absolution_position = ((recv3_2 << 16) | (recv3_3 << 8) | recv3_4)>> 5;
				if(absolution_position - (rt_int32_t)position_offset < 0){
					absolution_position = absolution_position - (rt_int32_t)position_offset + 524287;
				}
				else{
					absolution_position = absolution_position - (rt_int32_t)position_offset;
				}
				recv3_2 = ((rt_uint32_t)absolution_position & 0xff0000)>>16;
				recv3_3 = ((rt_uint32_t)absolution_position & 0xff00)>>8;
				recv3_4 = (rt_uint32_t)absolution_position & 0xff;
				if((recv4_2 & 0x01) == 1) //VALID,代表读取数据有效
				{
					Position_EW[0] = (recv3_2<<5) + (recv3_3 >>3);
					Position_EW[1] = (recv3_3<<5) + (recv3_4 >>3);
					Position_EW[2] = (recv3_4<<5) + 0x03;  //低两位为1代表错误位为1，警告位为1（错误位和警告位低电平有效）
				}
				else if(recv4_2 & 0x88)  //DISMISS,ERROR，发生错误
				{
					Position_EW[0] = (recv3_2<<5) + (recv3_3 >>3);
					Position_EW[1] = (recv3_3<<5) + (recv3_4 >>3);
					Position_EW[2] = (recv3_4<<5) + 0x01;  //低两位为1代表错误位为0，警告位为1（错误位和警告位低电平有效）	
				}
			}
			rt_sem_release(&tx_sem);
		}
		
//		//激活寄存器(RACTIVE)和传感器(PACTIVE) (OPCODE为0XB0)
//		struct rt_spi_message msg1,msg2,msg3,msg4;
//		msg1.send_buf   = &send1_1;
//		msg1.recv_buf   = &recv1_1;
//		msg1.length     = 1;
//		msg1.cs_take    = 1;
//		msg1.cs_release = 0;
//		msg1.next       = &msg2;
//		msg2.send_buf   = &send1_2;
//		msg2.recv_buf   = &recv1_2;
//		msg2.length     = 1;
//		msg2.cs_take    = 0;
//		msg2.cs_release = 1;
//		msg2.next       = RT_NULL;
//		rt_spi_transfer_message(spi_dev_recv, &msg1);
//		rt_kprintf("opcode1_1 is 0x%x\n",recv1_1);
//		rt_kprintf("opcode1_2 is 0x%x\n",recv1_2);
//		//读取SDAD-status (OPCODE为0XF5)
//		msg1.send_buf   = &send2_1;
//		msg1.recv_buf   = &recv2_1;
//		msg1.length     = 1;
//		msg1.cs_take    = 1;
//		msg1.cs_release = 0;
//		msg1.next       = &msg2;
//		msg2.send_buf   = &send2_2;
//		msg2.recv_buf   = &recv2_2;
//		msg2.length     = 1;
//		msg2.cs_take    = 0;
//		msg2.cs_release = 1;
//		msg2.next       = RT_NULL;
//		rt_spi_transfer_message(spi_dev_recv, &msg1);
//		rt_kprintf("opcode2_1 is 0x%x\n",recv2_1);
//		rt_kprintf("opcode2_2 is 0x%x\n",recv2_2);		
//		
////		//读寄存器
////		rt_uint8_t a1 = 0x97,a2 = 0x12,a3 = 0xad,a4 = 0x00,a5 = 0x00;
////		rt_uint8_t b1 = 0x00,b2 = 0x00,b3 = 0x00,b4 = 0x00,b5 = 0x00;
////		msg1.send_buf   = &a1;
////		msg1.recv_buf   = &b1;
////		msg1.length     = 1;
////		msg1.cs_take    = 1;
////		msg1.cs_release = 0;
////		msg1.next       = &msg2;
////		msg2.send_buf   = &a2;
////		msg2.recv_buf   = &b2;
////		msg2.length     = 1;
////		msg2.cs_take    = 0;
////		msg2.cs_release = 1;
////		msg2.next       = RT_NULL;
////		rt_spi_transfer_message(spi_dev_recv, &msg1);
////		
////		msg1.send_buf   = &a3;
////		msg1.recv_buf   = &b3;
////		msg1.length     = 1;
////		msg1.cs_take    = 1;
////		msg1.cs_release = 0;
////		msg1.next       = &msg2;
////		msg2.send_buf   = &a4;
////		msg2.recv_buf   = &b4;
////		msg2.length     = 1;
////		msg2.cs_take    = 1;
////		msg2.cs_release = 0;
////		msg2.next       = &msg3;
////		msg3.send_buf   = &a5;
////		msg3.recv_buf   = &b5;
////		msg3.length     = 1;
////		msg3.cs_take    = 0;
////		msg3.cs_release = 1;
////		msg3.next       = RT_NULL;
////		rt_spi_transfer_message(spi_dev_recv, &msg1);
////		rt_kprintf("b1 is 0x%x\n",b1);
////		rt_kprintf("b2 is 0x%x\n",b2);		
////		rt_kprintf("b3 is 0x%x\n",b3);
////		rt_kprintf("b4 is 0x%x\n",b4);	
////		rt_kprintf("b5 is 0x%x\n",b5);

//		//调零点功能，写寄存器（0x75）的值为0X03
//		rt_uint8_t zero_set_send1 = 0xD2;
//		rt_uint8_t zero_set_send2 = 0x75;
//		rt_uint8_t zero_set_send3 = 0x03;
//		rt_uint8_t zero_set_recv1 = 0x00;
//		rt_uint8_t zero_set_recv2 = 0x00;
//		rt_uint8_t zero_set_recv3 = 0x00;
//		msg1.send_buf   = &zero_set_send1;
//		msg1.recv_buf   = &zero_set_recv1;
//		msg1.length     = 1;
//		msg1.cs_take    = 1;
//		msg1.cs_release = 0;
//		msg1.next       = &msg2;
//		msg2.send_buf   = &zero_set_send2;
//		msg2.recv_buf   = &zero_set_recv2;
//		msg2.length     = 1;
//		msg2.cs_take    = 1;
//		msg2.cs_release = 0;
//		msg2.next       = &msg3;
//		msg3.send_buf   = &zero_set_send3;
//		msg3.recv_buf   = &zero_set_recv3;
//		msg3.length     = 1;
//		msg3.cs_take    = 0;
//		msg3.cs_release = 1; 
//		msg3.next       = RT_NULL;
//		rt_spi_transfer_message(spi_dev_recv, &msg1);
//		rt_kprintf("opcode_zero is 0x%x\n",zero_set_recv1);
//		rt_kprintf("zero_addr is 0x%x\n",zero_set_recv2);
//		rt_kprintf("zero_data is 0x%x\n",zero_set_recv3);
//		
//		//读取传感器数据（一共19位）
////		if(recv2_2 == 0x80)
////		{
////			while(1)
////			{
////				msg1.send_buf   = &send3_1;
////				msg1.recv_buf   = &recv3_1;
////				msg1.length     = 1;
////				msg1.cs_take    = 1;
////				msg1.cs_release = 0;
////				msg1.next       = &msg2;
////				msg2.send_buf   = &send3_2;
////				msg2.recv_buf   = &recv3_2;
////				msg2.length     = 1;
////				msg2.cs_take    = 1;
////				msg2.cs_release = 0;
////				msg2.next       = &msg3;
////				msg3.send_buf   = &send3_3;
////				msg3.recv_buf   = &recv3_3;
////				msg3.length     = 1;
////				msg3.cs_take    = 1;
////				msg3.cs_release = 0;
////				msg3.next       = &msg4;		
////				msg4.send_buf   = &send3_4;
////				msg4.recv_buf   = &recv3_4;
////				msg4.length     = 1;
////				msg4.cs_take    = 0;
////				msg4.cs_release = 1;
////				msg4.next       = NULL;					
////				rt_spi_transfer_message(spi_dev_recv, &msg1);
////				rt_kprintf("opcode3_1 is 0x%x\n",recv3_1);
////				rt_kprintf("opcode3_2 is 0x%x\n",recv3_2);
////				rt_kprintf("opcode3_3 is 0x%x\n",recv3_3);
////				rt_kprintf("opcode3_4 is 0x%x\n\n",recv3_4);
////				rt_uint32_t absolution_position = ((recv3_2 << 16) | (recv3_3 << 8) | recv3_4)>> 5;
////				rt_kprintf("abosolution_position is 0x%x\n",absolution_position);
////				int deg = (int)(360.0 * absolution_position/524287);
////				rt_kprintf("deg is %d\n\n",deg);
////				rt_thread_mdelay(2000);
////			}
////		}
}
int spi_recv_sample(void){
		rt_thread_t tid_recv;
		rt_err_t res;
		rt_err_t ret = RT_EOK;
	  /* 初始化信号量 */
    rt_sem_init(&rx_sem, "rx_sem", 0, RT_IPC_FLAG_FIFO);
		rt_sem_init(&tx_sem, "tx_sem", 0, RT_IPC_FLAG_FIFO);
		rt_sem_init(&spi_sem, "spi_sem", 0, RT_IPC_FLAG_FIFO);
		char name[RT_NAME_MAX];
    rt_strncpy(name, RECV_SPI_DEVICE_NAME, RT_NAME_MAX);
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
		else 
			ret = RT_ERROR;
		return ret;
}
INIT_APP_EXPORT(spi_recv_sample);
