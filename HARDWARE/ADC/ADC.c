#include "ADC.h"

//void ADC_IO_Init(void){
//	ADC_InitTypeDef ADC_InitStructure; 
//	GPIO_InitTypeDef GPIO_InitStructure;

//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE |RCC_APB2Periph_ADC1,ENABLE );		//使能ADC1通道时钟

//	RCC_ADCCLKConfig(RCC_PCLK2_Div6);		//设置ADC分频因子6 72M/6=12,ADC最大时间不能超过14M

//	//PA1 作为模拟通道输入引脚
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;		//模拟输入引脚
//	GPIO_Init(GPIOE, &GPIO_InitStructure);	

//	ADC_DeInit(ADC1);  //复位ADC1,将外设 ADC1 的全部寄存器重设为缺省值

//	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;	//ADC工作模式:ADC1和ADC2工作在独立模式
//	ADC_InitStructure.ADC_ScanConvMode = DISABLE;	//模数转换工作在单通道模式
//	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;	//模数转换工作在单次转换模式
//	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;	//转换由软件而不是外部触发启动
//	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//ADC数据右对齐
//	ADC_InitStructure.ADC_NbrOfChannel = 1;	//顺序进行规则转换的ADC通道的数目
//	ADC_Init(ADC1, &ADC_InitStructure);	//根据ADC_InitStruct中指定的参数初始化外设ADCx的寄存器   

//	ADC_Cmd(ADC1, ENABLE);	//使能指定的ADC1
//	
//	ADC_ResetCalibration(ADC1);	//使能复位校准  

//	while(ADC_GetResetCalibrationStatus(ADC1));	//等待复位校准结束
//	
//	ADC_StartCalibration(ADC1);	 //开启AD校准
// 
//	while(ADC_GetCalibrationStatus(ADC1));	 //等待校准结束
//	
//}

void MCU_Temperature_ADC_Init(void){
	ADC_InitTypeDef ADC_InitStructure; 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_ADC1,ENABLE );		//使能GPIOA,ADC1通道时钟
	
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);		//ADC的时钟不可以超过14Mhz

	ADC_DeInit(ADC1);		//将ADC1恢复默认值
 
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;		//ADC工作模式:ADC1和ADC2工作在独立模式
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;		//模数转换工作在单通道模式
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;		//模数转换工作在单次转换模式
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;		//转换由软件而不是外部触发启动
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;		//ADC数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = 1;		//顺序进行规则转换的ADC通道的数目
	ADC_Init(ADC1, &ADC_InitStructure);		//根据ADC_InitStruct中指定的参数初始化外设ADCx的寄存器

	ADC_TempSensorVrefintCmd(ENABLE);		//开启内部温度传感器
	
	ADC_Cmd(ADC1,ENABLE);		//使能ADC1

	ADC_ResetCalibration(ADC1);		//重置指定的ADC1的复位寄存器

  while(ADC_GetResetCalibrationStatus(ADC1));		//获取ADC1重置校准寄存器的状态,设置状态则等待

	ADC_StartCalibration(ADC1);

	while(ADC_GetCalibrationStatus(ADC1));		//获取指定ADC1的校准程序,设置状态则等待
	
}

static unsigned short T_Get_Adc(unsigned char ch)   
{
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_239Cycles5 );		//ADC1,ADC通道3,第一个转换,采样时间为239.5周期
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);		//使能指定的ADC1的软件转换启动功能
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));		//等待转换结束
	return ADC_GetConversionValue(ADC1);		//返回最近一次ADC1规则组的转换结果
}

////得到ADC采样内部温度传感器的值
////取10次,然后平均
//static unsigned short T_Get_Temp(void){
//	unsigned short temp_val=0;
//	unsigned char t;
//	for(t=0;t<10;t++){
//		temp_val+=T_Get_Adc(ADC_Channel_16);	  //TampSensor
//		delay_ms(5);
//	}
//	return temp_val/10;
//}

//获取通道ch的转换值
//取times次,然后平均
static unsigned short T_Get_Adc_Average(unsigned char ch,unsigned char times){
	u32 temp_val=0;
	unsigned char t;
	for(t=0;t<times;t++){
		temp_val+=T_Get_Adc(ch);
		delay_ms(5);
	}
	return temp_val/times;
}

//得到温度值
//返回值:温度值(扩大了100倍,单位:℃.)
short Get_Temprate(void)		//获取内部温度传感器温度值
{
	u32 adcx;
	short result;
 	double temperate;
	adcx=T_Get_Adc_Average(ADC_Channel_16,20);	//读取通道16,20次取平均
	temperate=(float)adcx*(3.3/4096);		//电压值 
	temperate=(1.43-temperate)/0.0043+25;		//转换为温度值 	 
	result=temperate*=100;
	return result;
}

