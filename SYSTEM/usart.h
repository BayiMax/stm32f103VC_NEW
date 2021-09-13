#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "sys.h" 

#define USART_REC_LEN  			200  	//�����������ֽ��� 200
#define EN_USART1_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����

extern u8  USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u16 USART_RX_STA;         		//����״̬���	
//����봮���жϽ��գ��벻Ҫע�����º궨��
void uart_init(u32 bound);

#define UART4_MAX_RECV_LEN		1024				//�����ջ����ֽ���
#define UART4_MAX_SEND_LEN		1024				//����ͻ����ֽ���
#define UART4_RX_EN 			1					//0,������;1,����.

extern u8  UART4_RX_BUF[UART4_MAX_RECV_LEN]; 		//���ջ���,���USART2_MAX_RECV_LEN�ֽ�
extern u8  UART4_TX_BUF[UART4_MAX_SEND_LEN]; 		//���ͻ���,���USART2_MAX_SEND_LEN�ֽ�
extern u16 UART4_RX_STA;   						//��������״̬

void TIM7_Init(void);
void TIM7_Set(u8 sta);


void UART4_Config(unsigned int bound);
void Uart4_SendByte(uint8_t ch);
void Uart4_SendString(char *str);

void UART4_DMA_Config(DMA_Channel_TypeDef*DMA_CHx,u32 cpar,u32 cmar);
void UART4_DMA_Enable(DMA_Channel_TypeDef*DMA_CHx,u8 len);

#endif


