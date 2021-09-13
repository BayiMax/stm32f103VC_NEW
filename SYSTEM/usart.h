#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "sys.h" 

#define USART_REC_LEN  			200  	//定义最大接收字节数 200
#define EN_USART1_RX 			1		//使能（1）/禁止（0）串口1接收

extern u8  USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern u16 USART_RX_STA;         		//接收状态标记	
//如果想串口中断接收，请不要注释以下宏定义
void uart_init(u32 bound);

#define UART4_MAX_RECV_LEN		1024				//最大接收缓存字节数
#define UART4_MAX_SEND_LEN		1024				//最大发送缓存字节数
#define UART4_RX_EN 			1					//0,不接收;1,接收.

extern u8  UART4_RX_BUF[UART4_MAX_RECV_LEN]; 		//接收缓冲,最大USART2_MAX_RECV_LEN字节
extern u8  UART4_TX_BUF[UART4_MAX_SEND_LEN]; 		//发送缓冲,最大USART2_MAX_SEND_LEN字节
extern u16 UART4_RX_STA;   						//接收数据状态

void TIM7_Init(void);
void TIM7_Set(u8 sta);


void UART4_Config(unsigned int bound);
void Uart4_SendByte(uint8_t ch);
void Uart4_SendString(char *str);

void UART4_DMA_Config(DMA_Channel_TypeDef*DMA_CHx,u32 cpar,u32 cmar);
void UART4_DMA_Enable(DMA_Channel_TypeDef*DMA_CHx,u8 len);

#endif


