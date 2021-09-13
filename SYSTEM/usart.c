#include "sys.h"
#include "usart.h"	  
#include "stm32f10x_dma.h"
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos 使用	  
#endif

//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{
	x = x;
}
//重定义fputc函数 
int fputc(int ch, FILE *f)
{
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕
  USART1->DR = (u8) ch;
	return ch;
}
#endif 

 
#if EN_USART1_RX   //如果使能了接收
//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	
u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART_RX_STA=0;       //接收状态标记	  
  
void uart_init(u32 bound){
  //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//使能USART1，GPIOA时钟
  
	//USART1_TX   GPIOA.9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.9
   
  //USART1_RX	  GPIOA.10初始化
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.10  

  //Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
  
   //USART 初始化设置

	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

  USART_Init(USART1, &USART_InitStructure); //初始化串口1
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启串口接受中断
  USART_Cmd(USART1, ENABLE);                    //使能串口1 

}

void USART1_IRQHandler(void)                	//串口1中断服务程序
	{
	u8 Res;
#if SYSTEM_SUPPORT_OS 		//如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
	OSIntEnter();    
#endif
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
		{
		Res =USART_ReceiveData(USART1);	//读取接收到的数据
		
		if((USART_RX_STA&0x8000)==0)//接收未完成
			{
			if(USART_RX_STA&0x4000)//接收到了0x0d
				{
				if(Res!=0x0a)USART_RX_STA=0;//接收错误,重新开始
				else USART_RX_STA|=0x8000;	//接收完成了 
				}
			else //还没收到0X0D
				{	
				if(Res==0x0d)USART_RX_STA|=0x4000;
				else
					{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//接收数据错误,重新开始接收	  
					}		 
				}
			}   		 
     } 
#if SYSTEM_SUPPORT_OS 	//如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
	OSIntExit();  											 
#endif
} 
#endif	
/********************************************************/
//串口发送缓存区 	
__align(8) u8 UART4_TX_BUF[UART4_MAX_SEND_LEN]; 	//发送缓冲,最大USART2_MAX_SEND_LEN字节

u8 UART4_RX_BUF[UART4_MAX_RECV_LEN];
u16 UART4_RX_STA=0; 

void UART4_IRQHandler(void) 
{
	u8 res;	    
	if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)//接收到数据
	{
		res =USART_ReceiveData(UART4);		
		if(UART4_RX_STA<UART4_MAX_RECV_LEN)		//还可以接收数据
		{
			TIM_SetCounter(TIM7,0);//计数器清空        				 
			if(UART4_RX_STA==0)TIM7_Set(1);	 	//使能定时器4的中断 
			UART4_RX_BUF[UART4_RX_STA++]=res;		//记录接收到的值	 
		}else 
		{
			UART4_RX_STA|=1<<15;					//强制标记接收完成
		} 
	}  							
}
static void TIM7_NVIC_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure; 
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);		
	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn ;	
	// 设置主优先级为 0
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;	 
	// 设置抢占优先级为3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

static void TIM7_Mode_Config(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
	TIM_TimeBaseStructure.TIM_Period = 1999;	
	TIM_TimeBaseStructure.TIM_Prescaler= 7199;
	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);
	TIM_ClearFlag(TIM7, TIM_FLAG_Update);
	TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE);
	TIM_Cmd(TIM7, ENABLE);	
}

void TIM7_Init(void)
{
	TIM7_NVIC_Config();
	TIM7_Mode_Config();
}
void TIM7_Set(u8 sta)
{
	if(sta)
	{
		TIM_SetCounter(TIM7,0);//计数器清空
		TIM_Cmd(TIM7, ENABLE);  //使能TIMx	
	}else TIM_Cmd(TIM7, DISABLE);//关闭定时器4	   
}
void TIM7_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET)//是更新中断
	{
		UART4_RX_STA|=1<<15;	//标记接收完成
		TIM_ClearITPendingBit(TIM7,TIM_IT_Update);  //清除TIMx更新中断标志    
		TIM7_Set(0);			//关闭TIM4  
	}	    
}
static void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;
  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  
  NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

void UART4_Config(unsigned int bound)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	USART_InitStructure.USART_BaudRate = bound;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(UART4, &USART_InitStructure);
	
	USART_DMACmd(UART4,USART_DMAReq_Tx,ENABLE);
	
	UART4_DMA_Config(DMA1_Channel5,(u32)&UART4->DR,(u32)UART4_TX_BUF);
	
	USART_Cmd(UART4,ENABLE);
	
	NVIC_Configuration();

	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);	

	USART_Cmd(UART4, ENABLE);	

	TIM7_Init();		//定时器初始化
	UART4_RX_STA=0;		//清零
	TIM7_Set(0);			//关闭定时器4
}

void UART4_DMA_Config(DMA_Channel_TypeDef*DMA_CHx,u32 cpar,u32 cmar)
{
	DMA_InitTypeDef DMA_InitStructure;
 	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);		//使能DMA传输
  DMA_DeInit(DMA_CHx);		//将DMA的通道1寄存器重设为缺省值
	DMA_InitStructure.DMA_PeripheralBaseAddr = cpar;		//DMA外设ADC基地址
	DMA_InitStructure.DMA_MemoryBaseAddr = cmar;		//DMA内存基地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;		//数据传输方向，从内存读取发送到外设
	DMA_InitStructure.DMA_BufferSize = 0;		//DMA通道的DMA缓存的大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;		//外设地址寄存器不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;		//内存地址寄存器递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;		//数据宽度为8位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;		//数据宽度为8位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;		//工作在正常缓存模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;		//DMA通道 x拥有中优先级 
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;		//DMA通道x没有设置为内存到内存传输
	DMA_Init(DMA_CHx, &DMA_InitStructure);		//根据DMA_InitStruct中指定的参数初始化DMA的通道USART1_Tx_DMA_Channel所标识的寄存器	
}
//开启一次DMA传输
void UART4_DMA_Enable(DMA_Channel_TypeDef*DMA_CHx,u8 len)
{
	DMA_Cmd(DMA_CHx, DISABLE );		//关闭 指示的通道
	DMA_SetCurrDataCounter(DMA_CHx,len);		//DMA通道的DMA缓存的大小
	DMA_Cmd(DMA_CHx, ENABLE);		//开启DMA传输
}

void Uart4_SendByte(unsigned char ch)
{
	UART4->DR = (ch & (uint16_t)0x01FF);
	while (USART_GetFlagStatus(UART4, USART_FLAG_TXE) == RESET);	
}

//---------------发送字符串---------------------/
void Uart4_SendString(char *Dat)
{
	unsigned int a=0;
  do 
  {
		Uart4_SendByte(*(Dat + a));
		a++;
  } while(*(Dat + a)!='\0');
  while(USART_GetFlagStatus(UART4,USART_FLAG_TC)==RESET){}
}



