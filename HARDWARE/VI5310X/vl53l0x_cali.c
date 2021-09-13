#include "vl53l0x_cali.h"
#include "main.h"
#if VL53L0X==1

_vl53l0x_adjust Vl53l0x_adjust; //校准数据24c02写缓存区(用于在校准模式校准数�?写入24c02)

_vl53l0x_adjust Vl53l0x_data;   //校准数据24c02读缓存区（用于系统初始化时向24C02读取数据�?

#define adjust_num 5//校准错�??次数

//VL53L0X校准函数
//dev:设�?�I2C参数结构�?
VL53L0X_Error vl53l0x_adjust(VL53L0X_Dev_t *dev)
{
	
	VL53L0X_DeviceError Status = VL53L0X_ERROR_NONE;
	uint32_t refSpadCount = 7;
	uint8_t  isApertureSpads = 0;
	uint32_t XTalkCalDistance = 100;
	uint32_t XTalkCompensationRateMegaCps;
	uint32_t CalDistanceMilliMeter = 100<<16;
	int32_t  OffsetMicroMeter = 30000;
	uint8_t VhvSettings = 23;
	uint8_t PhaseCal = 1;
	u8 i=0;

	VL53L0X_StaticInit(dev);//数值恢复默�?,传感器�?�于空闲状�?
	printf("The value is restored to the default value and the sensor is idle\r\n");
	//SPADS校准----------------------------
	spads:
	delay_ms(10);
	printf("SPADS Calibration start...\r\n");
	Status = VL53L0X_PerformRefSpadManagement(dev,&refSpadCount,&isApertureSpads);//执�?�参考Spad管理
	if(Status == VL53L0X_ERROR_NONE)
	{
		printf("refSpadCount = %d\r\n",refSpadCount);
		Vl53l0x_adjust.refSpadCount = refSpadCount;
		printf("isApertureSpads = %d\r\n",isApertureSpads);	
		Vl53l0x_adjust.isApertureSpads = isApertureSpads;
		printf("The SPADS Calibration Finish...\r\n\r\n");		
		i=0;
	}
	else
	{
		i++;
		if(i==adjust_num) return Status;
		printf("SPADS Calibration error, restart this step\r\n");
		goto spads;
	}
	//设�?�参考校�?---------------------------------------------------
	ref:
	delay_ms(10);
	printf("Start of reference correction...\r\n");
	Status = VL53L0X_PerformRefCalibration(dev,&VhvSettings,&PhaseCal);//Ref参考校�?
	if(Status == VL53L0X_ERROR_NONE)
	{
		printf("VhvSettings = %d\r\n",VhvSettings);
		Vl53l0x_adjust.VhvSettings = VhvSettings;
		printf("PhaseCal = %d\r\n",PhaseCal);
		Vl53l0x_adjust.PhaseCal = PhaseCal;
		printf("Reference calibration completed...\r\n\r\n");
		i=0;
	}
	else
	{
		i++;
		if(i==adjust_num) return Status;
		printf("Restart this step by referring to calibration errors\r\n");
		goto ref;
	}
	//偏移校准------------------------------------------------
	offset:
	delay_ms(10);
	printf("Offset calibration: need no strong light environment, 100mm from the white target\r\n");
	printf("Start of offset calibration�?\r\n");
	Status = VL53L0X_PerformOffsetCalibration(dev,CalDistanceMilliMeter,&OffsetMicroMeter);//偏移校准
	if(Status == VL53L0X_ERROR_NONE)
	{
		printf("CalDistanceMilliMeter = %d mm\r\n",CalDistanceMilliMeter);
		Vl53l0x_adjust.CalDistanceMilliMeter = CalDistanceMilliMeter;
		printf("OffsetMicroMeter = %d mm\r\n",OffsetMicroMeter);	
		Vl53l0x_adjust.OffsetMicroMeter = OffsetMicroMeter;
		printf("The Offset Calibration Finish...\r\n\r\n");
		i=0;
	}
	else
	{
		i++;
		if(i==adjust_num) return Status;
		printf("If the offset calibration is incorrect, restart this step\r\n");
		goto offset;
	}
	//串扰校准-----------------------------------------------------
	xtalk:
	delay_ms(20);
	printf("Crosstalk correction: a grey target is required\r\n");
	printf("Crosstalk correction begins...\r\n");	
	Status = VL53L0X_PerformXTalkCalibration(dev,XTalkCalDistance,&XTalkCompensationRateMegaCps);//串扰校准
	if(Status == VL53L0X_ERROR_NONE)
	{
		printf("XTalkCalDistance = %d mm\r\n",XTalkCalDistance);
		Vl53l0x_adjust.XTalkCalDistance = XTalkCalDistance;
		printf("XTalkCompensationRateMegaCps = %d\r\n",XTalkCompensationRateMegaCps);	
		Vl53l0x_adjust.XTalkCompensationRateMegaCps = XTalkCompensationRateMegaCps;
		printf("Crosstalk calibration is complete...\r\n\r\n");
		i=0;
	}
	else
	{
		i++;
		if(i==adjust_num) return Status;
		printf("Crosstalk calibration error, restart this step\r\n");
		goto xtalk;
	}
	GPIOA->BSRR=GPIO_Pin_1;
	printf("All calibration is done!\r\n");
	printf("Calibration successful!\r\n");

	Vl53l0x_adjust.adjustok = 0xAA;//校准成功
//	AT24CXX_Write(0,(u8*)&Vl53l0x_adjust,sizeof(_vl53l0x_adjust));//将校准数�?保存�?24c02
	memcpy(&Vl53l0x_data,&Vl53l0x_adjust,sizeof(_vl53l0x_adjust));//将校准数�?复制到Vl53l0x_data结构�?
	return Status;
}

//vl53l0x校准测试
//dev:设�?�I2C参数结构�?
void vl53l0x_calibration_test(VL53L0X_Dev_t *dev)
{
	VL53L0X_Error status = VL53L0X_ERROR_NONE;
	u8 key=0;
	u8 i=0;
	
	LCD_Fill(0,0,240,240,BLACK);
	
	LCDDebug(0,0,"need a white target,and the distance keep 100mm.");
	LCDDebug(0,16*2,"3: Return menu");
	LCDDebug(0,16*3,"1:   Calibration");
	while(1)
	{
		if(KEY_Flag==1)
		{
			LCDDebug(0,16*4,"Start calibration...");
			status = vl53l0x_adjust(dev);//进入校准
			if(status!=VL53L0X_ERROR_NONE)//校准失败
			{
				printf("Calibration is error!!\r\n");
				i=3; 
				while(i--)
				{
					delay_ms(500);
					LCDDebug(0,16*5,"                    ");
					delay_ms(500);
					LCDDebug(0,16*6,"Calibration is error");
				}
			}
			else
				 LCDDebug(0,16*7,"Calibration is complete!");
			delay_ms(500);

			break;
				
		 }
		 else if(key==3)
		 {
			 GPIOA->BSRR=GPIO_Pin_1;
			 break;//返回上一菜单
		 }
		 delay_ms(200);
		 GPIOA->ODR^=GPIO_Pin_2;
	}
}


#endif 
