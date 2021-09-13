#include "led.h"
#include "main.h"

void RGB_IO_Init(void){
	GPIO_InitTypeDef RGB_IO_InitTypedef;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);//Ê¹ÄÜPORTA,PORTEÊ±ÖÓ
	
	RGB_IO_InitTypedef.GPIO_Pin=GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
	RGB_IO_InitTypedef.GPIO_Mode=GPIO_Mode_Out_PP;
	RGB_IO_InitTypedef.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&RGB_IO_InitTypedef);
	GPIOA->BSRR=GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3;
}
