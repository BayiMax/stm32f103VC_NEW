#include "main.h"
#include "key.h"

unsigned char KEY_Flag=0;

void Key_IO_Init(void){
	GPIO_InitTypeDef KEY_IO_Init;
	EXTI_InitTypeDef KEY_EXTI_Init;
	NVIC_InitTypeDef KEY_NVIC_Init;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE,ENABLE);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);	//ʹ�ܸ��ù���ʱ��
	
	KEY_IO_Init.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14;
	KEY_IO_Init.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(GPIOE, &KEY_IO_Init);
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource12);
	KEY_EXTI_Init.EXTI_Line=EXTI_Line12;
	KEY_EXTI_Init.EXTI_Mode = EXTI_Mode_Interrupt;
	KEY_EXTI_Init.EXTI_Trigger = EXTI_Trigger_Rising;		//�����ش���
	KEY_EXTI_Init.EXTI_LineCmd = ENABLE;
	EXTI_Init(&KEY_EXTI_Init);		//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource13);
	KEY_EXTI_Init.EXTI_Line=EXTI_Line13;
	KEY_EXTI_Init.EXTI_Mode = EXTI_Mode_Interrupt;
	KEY_EXTI_Init.EXTI_Trigger = EXTI_Trigger_Rising;		//�����ش���
	KEY_EXTI_Init.EXTI_LineCmd = ENABLE;
	EXTI_Init(&KEY_EXTI_Init);		//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���
	
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOE,GPIO_PinSource14);
	KEY_EXTI_Init.EXTI_Line=EXTI_Line14;
	KEY_EXTI_Init.EXTI_Mode = EXTI_Mode_Interrupt;
	KEY_EXTI_Init.EXTI_Trigger = EXTI_Trigger_Rising;		//�����ش���
	KEY_EXTI_Init.EXTI_LineCmd = ENABLE;
	EXTI_Init(&KEY_EXTI_Init);		//����EXTI_InitStruct��ָ���Ĳ�����ʼ������EXTI�Ĵ���
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	KEY_NVIC_Init.NVIC_IRQChannel = EXTI15_10_IRQn;			//ʹ�ܰ���E12���ڵ��ⲿ�ж�ͨ��
	KEY_NVIC_Init.NVIC_IRQChannelPreemptionPriority = 0x01;	//��ռ���ȼ�1
	KEY_NVIC_Init.NVIC_IRQChannelSubPriority = 0x01;					//�����ȼ�1
	KEY_NVIC_Init.NVIC_IRQChannelCmd = ENABLE;								//ʹ���ⲿ�ж�ͨ��
	NVIC_Init(&KEY_NVIC_Init); 
	
}


void EXTI15_10_IRQHandler(void){
	delay_ms(10);
	if(EXTI_GetITStatus(EXTI_Line12)==1){
		KEY_Flag=1;
	}
	if(EXTI_GetITStatus(EXTI_Line13)==1){
		KEY_Flag=2;
	}
	if(EXTI_GetITStatus(EXTI_Line14)==1){
		KEY_Flag=3;
	}
	EXTI_ClearITPendingBit(EXTI_Line12); //���LINE12�ϵ��жϱ�־λ 
	EXTI_ClearITPendingBit(EXTI_Line13); //���LINE13�ϵ��жϱ�־λ 
	EXTI_ClearITPendingBit(EXTI_Line14); //���LINE14�ϵ��жϱ�־λ 	
}
