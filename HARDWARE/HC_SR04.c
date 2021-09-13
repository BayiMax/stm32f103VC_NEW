#include "HC_SR04.h"
#include "main.h"

#if HC_SR04==1

unsigned char HC_SR04_Error_Flag=0;

unsigned char HC_SR04_Init(void){
    unsigned short Time_Out_Flag=0;
    TIM_TimeBaseInitTypeDef TIM7_TypeDef;
    GPIO_InitTypeDef HCSR04_GPIO_TYpeDef;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE,ENABLE);
    
    TIM7_TypeDef.TIM_Period = 65535;
    TIM7_TypeDef.TIM_Prescaler = 71;
    // TIM7_TypeDef.TIM_ClockDivision = TIM_CKD_DIV1;
    // TIM7_TypeDef.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM7,&TIM7_TypeDef);
    TIM_Cmd(TIM7, ENABLE);

    HCSR04_GPIO_TYpeDef.GPIO_Pin = GPIO_Pin_11;
    HCSR04_GPIO_TYpeDef.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOE,&HCSR04_GPIO_TYpeDef);      //echo
    GPIOE->BRR=GPIO_Pin_11;
    HCSR04_GPIO_TYpeDef.GPIO_Pin = GPIO_Pin_10;
    HCSR04_GPIO_TYpeDef.GPIO_Mode = GPIO_Mode_Out_PP;
    HCSR04_GPIO_TYpeDef.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOE,&HCSR04_GPIO_TYpeDef);      //trig
    GPIOE->BRR=GPIO_Pin_10;
    delay_ms(100);
    GPIOE->BSRR=GPIO_Pin_10;
    delay_ms(1);
    GPIOE->BRR=GPIO_Pin_10;
    TIM7->CNT=0;
    while (GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_11)==0){
        Time_Out_Flag++;
        if(Time_Out_Flag>5000){
            HC_SR04_Error_Flag=0xff;
            return 1;
        }
    }
    if(Time_Out_Flag<4999){
        return 0;
    }
    Time_Out_Flag=0;
    while(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_11)==1){
        Time_Out_Flag++;
        if(Time_Out_Flag>5000){
            HC_SR04_Error_Flag=0xff;
            return 1;
        }
    }
    if(Time_Out_Flag<4999){
        return 0;
    }
    return 0;
}

unsigned short Get_One_distance(void){
    unsigned short TimeOut_Flag=0;
    GPIOE->BSRR=GPIO_Pin_10;
    delay_ms(1);
    GPIOE->BRR=GPIO_Pin_10;
    TIM7->CNT=0;
    while (GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_11)==0){
        TimeOut_Flag++;
        if(TimeOut_Flag>6000){
             HC_SR04_Error_Flag=0xff;
             break;
        }
    }
    // TIM7->CR1|=0x00000001;
    TIM_Cmd(TIM7,ENABLE);
    TimeOut_Flag=0;
    while(GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_11)==1){
        TimeOut_Flag++;
        if(TimeOut_Flag>6000){
            HC_SR04_Error_Flag=0xff;
            break;
        }
    }
    //  TIM7->CR1|=0x00000002;
    TIM_Cmd(TIM7,DISABLE);
    return 17*(TIM7->CNT)/100;
}

unsigned short Get_Distance(void){
    unsigned char i;
    unsigned short arr[10];
    unsigned short distance=0;
    for(i=0;i<10;i++){
        arr[i]=Get_One_distance();
        delay_ms(20);
    }
    shell_sort(arr,10);
    arr[0]=0;arr[9]=0;
    for(i=1;i<9;i++){
        distance+=arr[i];
        delay_ms(10);
        // printf("distance:%d\r\n",distance);
    }
    distance=distance/8;
    // printf("distance:%d",distance);
    return distance;
}

#endif

