/****************************************************************************
 *	@笔者	：	Q
 *	@日期	：	2023年2月8日
 *	@所属	：	杭州友辉科技
 *	@功能	：	存放usart串口相关的函数
 *	@函数列表:
 *	1.	void uart1_init(u32 baud) -- 初始化串口1
 *	2.	void uart3_init(u32 baud) -- 初始化串口3
 *	3.	void uart1_send_byte(u8 dat) -- 串口1发送字节
 *	4.	void uart3_send_byte(u8 dat) -- 串口3发送字节
 *	5.	void uart1_send_str(char *s) -- 串口1发送字符串
 *	6.	void uart3_send_str(char *s) -- 串口3发送字符串
 ****************************************************************************/

#include "./usart/y_usart.h"

u8 uart_receive_buf[UART_BUF_SIZE];
uint16_t uart1_get_ok;

/* 初始化串口1 */
void uart1_init(uint32_t BaudRate)
{
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* 使能端口时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);

	USART_DeInit(USART1);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;		/* PA.9 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; /* 复用推挽输出 */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; /* 浮空输入 */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = BaudRate;									/* 串口波特率 */
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;						/* 字长为8位数据格式 */
	USART_InitStructure.USART_StopBits = USART_StopBits_1;							/* 字长为8位数据格式 */
	USART_InitStructure.USART_Parity = USART_Parity_No;								/* 无奇偶校验位 */
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;					/* 收发模式 */
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; /* 无硬件数据流控制 */
	USART_Init(USART1, &USART_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; /* 抢占优先级 */
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		  /* 子优先级 */
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			  /* IRQ通道使能 */
	NVIC_Init(&NVIC_InitStructure);

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); /* 开启串口接受中断 */
	USART_ITConfig(USART1, USART_IT_TXE, DISABLE); /* 禁止串口发送中断 */

	USART_Cmd(USART1, ENABLE); /* 使能串口1  */
}

/***********************************************
	功能介绍：	串口1发送字节
	函数参数：	dat 发送的字节
	返回值：		无
 ***********************************************/
void uart1_send_byte(u8 dat)
{
	USART_SendData(USART1, dat);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET)
		;
}

/***********************************************
	功能介绍：	串口1发送字符串
	函数参数：	*s 发送的字符串
	返回值：		无
 ***********************************************/
void uart1_send_str(u8 *s)
{
	while (*s)
	{
		uart1_send_byte(*s++);
	}
}

/***********************************************
	功能介绍：	串口1发送数字
	函数参数：	tmp 发送的数字
	返回值：		无
 ***********************************************/
void uart1_send_int(int tmp)
{
	static u8 str[20];
	sprintf((char *)str, "%d", tmp);
	uart1_send_str(str);
}

/* 重定义fputc函数,写这个函数可以使用printf,记得开启Use MicroLIB */
int fputc(int ch, FILE *f)
{
	while ((USART1->SR & 0X40) == 0)
		; // 循环发送,直到发送完毕
	USART1->DR = (u8)ch;
	return ch;
}

/* 串口1中断服务程序 */
void USART1_IRQHandler(void) /* 最后数据发送\r\n结束 */
{
	u8 sbuf_bak;

	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) // 接收中断
	{
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
		sbuf_bak = USART_ReceiveData(USART1);

		// uart1_send_byte(sbuf_bak);			  /* 数据回显 */
		sbuf_bak = USART_ReceiveData(USART1); // 读取接收到的数据
		if ((uart1_get_ok & 0x8000) == 0)	  // 接收未完成
		{

			if (sbuf_bak == '!')
			{
				uart1_get_ok |= 0x8000;
				uart_receive_buf[uart1_get_ok & 0X3FFF] = sbuf_bak;
				uart_receive_buf[(uart1_get_ok & 0X3FFF) + 1] = '\0';
			}
			else
			{
				uart_receive_buf[uart1_get_ok & 0X3FFF] = sbuf_bak;
				uart1_get_ok++;
				if (uart1_get_ok > (UART_BUF_SIZE - 1))
					uart1_get_ok = 0; // 接收数据错误,重新开始接收
			}
		}
	}
}
