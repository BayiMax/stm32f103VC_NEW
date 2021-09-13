#include "vl53l0x_it.h"
#include "main.h"

#if VL53L0X==1
//�����޾���ֵ ��λ:mm
#define Thresh_Low  60
#define Thresh_High 150

//�ж�ģʽ�����ṹ��
typedef struct 
{
     const int VL53L0X_Mode;//ģʽ
	 uint32_t ThreshLow;    //����ֵ
	 uint32_t ThreshHigh;   //����ֵ
}AlrmMode_t; 

AlrmMode_t AlarmModes ={
	
     VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_OUT,// value < thresh_low OR value > thresh_high
	 Thresh_Low<<16,
	 Thresh_High<<16
};

//�ж����ó�ʼ��
static void exti_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
 	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD,ENABLE);		 //ʹ��PORTAʱ��		
	/*GPIO1--PD1*/
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;				 //���ó���������
 	GPIO_Init(GPIOD, &GPIO_InitStructure);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOD,GPIO_PinSource1);
	EXTI_InitStructure.EXTI_Line = EXTI_Line1; 
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;		//�½��ش���
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);
	
    NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02;
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x03;
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure); 
}

//������־λ alarm_flag 
//1:�о���
//0����
unsigned char alarm_flag=0;

//�ⲿ�жϷ�����
void EXTI1_IRQHandler(void)
{
	alarm_flag=1;//��־
	EXTI_ClearITPendingBit(EXTI_Line1);  //���LINE4�ϵ��жϱ�־λ 
}

extern uint8_t AjustOK;
extern mode_data Mode_data[];

//vl53l0x�жϲ���ģʽ����
//dev:�豸I2C�����ṹ��
//mode: 0:Ĭ��;1:�߾���;2:������;3:����
void vl53l0x_interrupt_start(VL53L0X_Dev_t *dev,uint8_t mode)
{
	uint8_t VhvSettings;
	uint8_t PhaseCal;
	uint32_t refSpadCount;
	uint8_t isApertureSpads;
	VL53L0X_RangingMeasurementData_t RangingMeasurementData;
	static char buf[VL53L0X_MAX_STRING_LENGTH];//����ģʽ�ַ����ַ�������
	VL53L0X_Error status=VL53L0X_ERROR_NONE;//����״̬


	exti_init();//�жϳ�ʼ��
	mode_string(mode,buf);//��ʾ��ǰ���õ�ģʽ
	LCD_Fill(0,0,240,240,BLACK);
	LCDDebug(0,0,"Interrupt Mode             ");
	LCDDebug(0,16,"KEY_UP: Exit the test       ");
	LCDDebug(0,16*2,"Mode:        ");
	LCDDebug(0,16*3,(char*)buf);
	sprintf((char*)buf,"Thresh Low:  %d mm ",Thresh_Low);	
	LCDDebug(0,16*4,(char*)buf); 
	sprintf((char*)buf,"Thresh High: %d mm",Thresh_High);	
	LCDDebug(0,16*5,(char*)buf);
	LCDDebug(0,16*6,"Now value:     mm");

	vl53l0x_reset(dev);//��λvl53l0x(Ƶ���л�����ģʽ���׵��²ɼ��������ݲ�׼���������һ����)
	status = VL53L0X_StaticInit(dev);
	if(status!=VL53L0X_ERROR_NONE) goto error;

	if(AjustOK!=0)//��У׼����,д��У׼ֵ
	{
		status= VL53L0X_SetReferenceSpads(dev,Vl53l0x_data.refSpadCount,Vl53l0x_data.isApertureSpads);//�趨SpadsУ׼ֵ
		if(status!=VL53L0X_ERROR_NONE) goto error;
        delay_ms(2);
		status= VL53L0X_SetRefCalibration(dev,Vl53l0x_data.VhvSettings,Vl53l0x_data.PhaseCal);//�趨RefУ׼ֵ
		if(status!=VL53L0X_ERROR_NONE) goto error;
		delay_ms(2);
		status=VL53L0X_SetOffsetCalibrationDataMicroMeter(dev,Vl53l0x_data.OffsetMicroMeter);//�趨ƫ��У׼ֵ
		if(status!=VL53L0X_ERROR_NONE) goto error;
		delay_ms(2);
		status=VL53L0X_SetXTalkCompensationRateMegaCps(dev,Vl53l0x_data.XTalkCompensationRateMegaCps);//�趨����У׼ֵ
		if(status!=VL53L0X_ERROR_NONE) goto error; 
	 }
	 else
	 {
	 	status = VL53L0X_PerformRefCalibration(dev, &VhvSettings, &PhaseCal);//Ref�ο�У׼
		if(status!=VL53L0X_ERROR_NONE) goto error;
		delay_ms(2);
		status = VL53L0X_PerformRefSpadManagement(dev, &refSpadCount, &isApertureSpads);//ִ�вο�SPAD����
		if(status!=VL53L0X_ERROR_NONE) goto error;
        delay_ms(2);		 
	 }
	 status = VL53L0X_SetDeviceMode(dev,VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);//ʹ����������ģʽ
	 if(status!=VL53L0X_ERROR_NONE) goto error;
	 delay_ms(2);
	 status = VL53L0X_SetInterMeasurementPeriodMilliSeconds(dev,Mode_data[mode].timingBudget);//�����ڲ����ڲ���ʱ��
	 if(status!=VL53L0X_ERROR_NONE) goto error;
	 delay_ms(2);
	 status = VL53L0X_SetLimitCheckEnable(dev,VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,1);//ʹ��SIGMA��Χ���
	 if(status!=VL53L0X_ERROR_NONE) goto error;
	 delay_ms(2);
	 status = VL53L0X_SetLimitCheckEnable(dev,VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,1);//ʹ���ź����ʷ�Χ���
	 if(status!=VL53L0X_ERROR_NONE) goto error;
	 delay_ms(2);
	 status = VL53L0X_SetLimitCheckValue(dev,VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,Mode_data[mode].sigmaLimit);//�趨SIGMA��Χ
	 if(status!=VL53L0X_ERROR_NONE) goto error;
	 delay_ms(2); 
	 status = VL53L0X_SetLimitCheckValue(dev,VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,Mode_data[mode].signalLimit);//�趨�ź����ʷ�Χ��Χ
	 if(status!=VL53L0X_ERROR_NONE) goto error; 
	 delay_ms(2);
	 status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(dev,Mode_data[mode].timingBudget);//�趨��������ʱ��
	 if(status!=VL53L0X_ERROR_NONE) goto error;
	 delay_ms(2); 
	 status = VL53L0X_SetVcselPulsePeriod(dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, Mode_data[mode].preRangeVcselPeriod);//�趨VCSEL��������
	 if(status!=VL53L0X_ERROR_NONE) goto error;
	 delay_ms(2);
	 status = VL53L0X_SetVcselPulsePeriod(dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, Mode_data[mode].finalRangeVcselPeriod);//�趨VCSEL�������ڷ�Χ
	 if(status!=VL53L0X_ERROR_NONE) goto error;
	 delay_ms(2);
	 status = VL53L0X_StopMeasurement(dev);//ֹͣ����
	 if(status!=VL53L0X_ERROR_NONE) goto error;
	 delay_ms(2);
	 status = VL53L0X_SetInterruptThresholds(dev,VL53L0X_DEVICEMODE_CONTINUOUS_RANGING,AlarmModes.ThreshLow, AlarmModes.ThreshHigh);//�趨�����ж��ϡ�����ֵ
	 if(status!=VL53L0X_ERROR_NONE) goto error;
	 delay_ms(2);
	 status = VL53L0X_SetGpioConfig(dev,0,VL53L0X_DEVICEMODE_CONTINUOUS_RANGING,AlarmModes.VL53L0X_Mode,VL53L0X_INTERRUPTPOLARITY_LOW);//�趨�����ж�ģʽ �½���
	 if(status!=VL53L0X_ERROR_NONE) goto error;
	 delay_ms(2);
	 status = VL53L0X_ClearInterruptMask(dev,0);//���VL53L0X�жϱ�־λ
	 
	 error://������Ϣ
	 if(status!=VL53L0X_ERROR_NONE)
	 {
		print_pal_error(status);
		return ;
	 }

	 alarm_flag = 0;
	 VL53L0X_StartMeasurement(dev);//��������
	 while(1)
	 {
		if(KEY_Flag==1)
		{
			VL53L0X_ClearInterruptMask(dev,0);//���VL53L0X�жϱ�־λ
			status = VL53L0X_StopMeasurement(dev); //ֹͣ����
			GPIOA->ODR^=GPIO_Pin_1;
			break;//������һ�˵�
		}
		if(alarm_flag==1)//�����ж�
		{
			alarm_flag=0;
			VL53L0X_GetRangingMeasurementData(dev,&RangingMeasurementData);//��ȡ��������,������ʾ����
			printf("d: %3d mm\r\n",RangingMeasurementData.RangeMilliMeter);
			LCD_ShowIntNum(0,16,RangingMeasurementData.RangeMilliMeter,4,WHITE,BLACK,16);
			delay_ms(70);
			VL53L0X_ClearInterruptMask(dev,0);//���VL53L0X�жϱ�־λ 
			
		}
		delay_ms(30);
		GPIOA->ODR^=GPIO_Pin_1;

	 }
}

//vl53l0x�жϲ���ģʽUI
void interrupt_ui(void)
{
//	LCD_Fill(30,140+20,300,300,WHITE);
//	POINT_COLOR=RED;        //��������Ϊ��ɫ 
//	LCD_ShowString(30,140+30,300,16,16,"Interrupt Mode                  ");
//	LCD_ShowString(30,140+55,300,16,16,"KEY1: Switch working mode    ");
//	POINT_COLOR=BLUE;       //��������Ϊ��ɫ 
//	LCD_ShowString(30,140+75,300,16,16, "KEY_UP: Return menu    ");
//	LCD_ShowString(30,140+95,300,16,16, "KEY0:   Default        ");
	
}

//vl53l0x�жϲ���ģʽ����
//dev:�豸I2C�����ṹ��
void vl53l0x_interrupt_test(VL53L0X_Dev_t *dev)
{
	u8 i=0;
	u8 mode=0;
	GPIOA->BSRR=GPIO_Pin_1;
	interrupt_ui();//��ʾ�жϲ���ģʽUI
	
	while(1)
	{
		
		if(KEY_Flag==1)	break;//�������˵� 	
		
		else if(KEY_Flag==2)//ѡ����ģʽ
		{
       mode++;
			 if(mode==4) mode=0;
			
			 switch(mode)
			 {
				 case Default_Mode:  LCDDebug(0,0, "Default        "); break;//Ĭ��
				 case HIGH_ACCURACY: LCDDebug(0,16*2, "High Accuracy  "); break;//�߾���
				 case LONG_RANGE:    LCDDebug(0,16*3, "Long Range     "); break;//������
				 case HIGH_SPEED:    LCDDebug(0,16*4, "High Speed     "); break;//����
			 }
		}	
		else if(KEY_Flag==3)//��������
		{
			vl53l0x_interrupt_start(dev,mode);
	    interrupt_ui();
			mode=0;
		}			
		i++;
		if(i==5)
		{
			i=0;
		}
		delay_ms(50);
	}
	
}

#endif
