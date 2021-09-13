#include "vl53l0x.h"
#include "main.h"

#if VL53L0X==1


VL53L0X_Dev_t vl53l0x_dev;//设备I2C数据参数
VL53L0X_DeviceInfo_t vl53l0x_dev_info;//设备ID版本信息
uint8_t AjustOK=0;//校准标志位

//VL53L0X各精度模式参数
//0：默认;1:高精度;2:长距离;3:高速
mode_data Mode_data[]=
{
    {(FixPoint1616_t)(0.25*65536), 
	 (FixPoint1616_t)(18*65536),
	 33000,
	 14,
	 10},//默认
		
	{(FixPoint1616_t)(0.25*65536) ,
	 (FixPoint1616_t)(18*65536),
	 200000, 
	 14,
	 10},//高精度
		
    {(FixPoint1616_t)(0.1*65536) ,
	 (FixPoint1616_t)(60*65536),
	 33000,
	 18,
	 14},//长距离
	
    {(FixPoint1616_t)(0.25*65536) ,
	 (FixPoint1616_t)(32*65536),
	 20000,
	 14,
	 10},//高速
		
};

//API错误信息打印
//Status：详情看VL53L0X_Error参数的定义
void print_pal_error(VL53L0X_Error Status)
{
	char buf[VL53L0X_MAX_STRING_LENGTH];
	VL53L0X_GetPalErrorString(Status,buf);//根据Status状态获取错误信息字符串
	// sprintf(sprintf_debug_buf,"API Status: %i : %s",Status, buf);
	// LCDDebug(0,16,sprintf_debug_buf);
	printf("API Status: %i : %s\r\n",Status, buf);//打印状态和错误信息
	
}

//模式字符串显示
//mode:0-默认;1-高精度;2-长距离;3-高速
void mode_string(u8 mode,char *buf)
{
	switch(mode)
	{
		case Default_Mode: strcpy(buf,"Default");        break;
		case HIGH_ACCURACY: strcpy(buf,"High Accuracy"); break;
		case LONG_RANGE: strcpy(buf,"Long Range");       break;
		case HIGH_SPEED: strcpy(buf,"High Speed");       break;
	}

}

//配置VL53L0X设备I2C地址
//dev:设备I2C参数结构体
//newaddr:设备新I2C地址
VL53L0X_Error vl53l0x_Addr_set(VL53L0X_Dev_t *dev,uint8_t newaddr)
{
	uint16_t Id;
	uint8_t FinalAddress;
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	u8 sta=0x00;
	
	FinalAddress = newaddr;
	
	if(FinalAddress==dev->I2cDevAddr)//新设备I2C地址与旧地址一致,直接退出
		return VL53L0X_ERROR_NONE;
	//在进行第一个寄存器访问之前设置I2C标准模式(400Khz)
	Status = VL53L0X_WrByte(dev,0x88,0x00);
	if(Status!=VL53L0X_ERROR_NONE) 
	{
		sta=0x01;//设置I2C标准模式出错
		goto set_error;
	}
	//尝试使用默认的0x52地址读取一个寄存器
	Status = VL53L0X_RdWord(dev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id);
	if(Status!=VL53L0X_ERROR_NONE) 
	{
		sta=0x02;//读取寄存器出错
		goto set_error;
	}
	if(Id == 0xEEAA)
	{
		//设置设备新的I2C地址
		Status = VL53L0X_SetDeviceAddress(dev,FinalAddress);
		if(Status!=VL53L0X_ERROR_NONE) 
		{
			sta=0x03;//设置I2C地址出错
			goto set_error;
		}
		//修改参数结构体的I2C地址
		dev->I2cDevAddr = FinalAddress;
		//检查新的I2C地址读写是否正常
		Status = VL53L0X_RdWord(dev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &Id);
		if(Status!=VL53L0X_ERROR_NONE) 
		{
			sta=0x04;//新I2C地址读写出错
			goto set_error;
		}	
	}
	set_error:
	if(Status!=VL53L0X_ERROR_NONE)
	{
		print_pal_error(Status);//打印错误信息
	}
	if(sta!=0)
	  printf("sta:0x%x\r\n",sta);
	return Status;
}

//vl53l0x复位函数
//dev:设备I2C参数结构体
void vl53l0x_reset(VL53L0X_Dev_t *dev)
{
	uint8_t addr;
	addr = dev->I2cDevAddr;//保存设备原I2C地址
    VL53L0X_Xshut(0);//失能VL53L0X
	delay_ms(30);
	VL53L0X_Xshut(1);//使能VL53L0X,让传感器处于工作(I2C地址会恢复默认0X52)
	delay_ms(30);
	dev->I2cDevAddr=0x52;
	vl53l0x_Addr_set(dev,addr);//设置VL53L0X传感器原来上电前原I2C地址
	VL53L0X_DataInit(dev);	
}

//初始化vl53l0x
//dev:设备I2C参数结构体
VL53L0X_Error vl53l0x_init(VL53L0X_Dev_t *dev)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	VL53L0X_Error Status = VL53L0X_ERROR_NONE;
	VL53L0X_Dev_t *pMyDevice = dev;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD,ENABLE);   //先使能外设IO PORTA时钟 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;		//端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;       //推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;      //IO口速度为50MHz
	GPIO_Init(GPIOD, &GPIO_InitStructure);				   //根据设定参数初始化GPIOA

	pMyDevice->I2cDevAddr = VL53L0X_Addr;//I2C地址(上电默认0x52)
	pMyDevice->comms_type = 1;           //I2C通信模式
	pMyDevice->comms_speed_khz = 400;    //I2C通信速率
	
	VL53L0X_i2c_init();//初始化IIC总线
	
	VL53L0X_Xshut(0);//失能VL53L0X
	delay_ms(30);
	VL53L0X_Xshut(1);//使能VL53L0X,让传感器处于工作
	delay_ms(30);
	
	vl53l0x_Addr_set(pMyDevice,0x54);		//设置VL53L0X传感器I2C地址
	if(Status!=VL53L0X_ERROR_NONE) goto error;
	Status = VL53L0X_DataInit(pMyDevice);//设备初始化
	if(Status!=VL53L0X_ERROR_NONE) goto error;
	delay_ms(2);
	Status = VL53L0X_GetDeviceInfo(pMyDevice,&vl53l0x_dev_info);//获取设备ID信息
	if(Status!=VL53L0X_ERROR_NONE) goto error;
	
//	AT24CXX_Read(0,(u8*)&Vl53l0x_data,sizeof(_vl53l0x_adjust));//读取24c02保存的校准数据,若已校准 Vl53l0x_data.adjustok==0xAA
//	if(Vl53l0x_data.adjustok==0xAA)//已校准
//	  AjustOK=1;
//	else //没校准

	AjustOK=0;		//直接标识为未校准
	
	error:
	if(Status!=VL53L0X_ERROR_NONE)
	{
		print_pal_error(Status);//打印错误信息
		return Status;
	}
  	
	return Status;
}

//主菜单界面
void vl53l0x_mtest_ui(void)
{
	 LCDDebug(0,0,"1: Calibration mode");//校准模式
	 LCDDebug(0,16,"2:   General mode");    //普通测量模式
	 LCDDebug(0,16*2,"3:   Interrupt mode");  //中断测量模式
}

//VL53L0X主测试程序
void vl53l0x_test(void)
{   
	while(vl53l0x_init(&vl53l0x_dev))//vl53l0x初始化
	{
		LCDDebug(0,0,"VL53L0X Error!!!");
		delay_ms(500);
		LCDDebug(0,0,"                ");
		delay_ms(500);
		GPIOA->ODR^=GPIO_Pin_1;
	}
	printf("VL53L0X OK\r\n");
	LCDDebug(0,16,"VL53L0X OK");
	vl53l0x_mtest_ui();//主菜单显示
	while(1)
	{
		switch(KEY_Flag)
		{
			case 1:  vl53l0x_calibration_test(&vl53l0x_dev);           break;//校准模式
			case 2:  vl53l0x_general_test(&vl53l0x_dev);               break;//普通测量模式
			case 3:  vl53l0x_interrupt_test(&vl53l0x_dev);             break;//中断测量模式  
		}
		vl53l0x_mtest_ui();
	}
}

void Vl53L0X_Init(void){
	unsigned char Time_Out_Flag=0;
	while(vl53l0x_init(&vl53l0x_dev)){
		printf("Vl53l0x is error   \r\n");
		LCDDebug(0,0,"Vl53l0x is error   ");
		delay_ms(200);
		LCDDebug(0,0,"                   ");
		delay_ms(200);
		Time_Out_Flag++;
		if(Time_Out_Flag>5){
			printf("vl53l0x is no ,break    \r\n");
			LCDDebug(0,0,"vl53l0x is no,break     ");
			break;
		}
	}
	if(Time_Out_Flag<4){
		printf("Vl53l0x INit is Ok\r\n");
		LCDDebug(0,0,"Vl53l0x INit is Ok         ");
	}
}
//VL53L0X普通模式测量
void VL53L0X_Get_Common(unsigned char mode){
	unsigned char i;
	char D_x[32];
	while(vl53l0x_set_mode(&vl53l0x_dev,mode))//配置精度模式
	{
		printf("Mode Set Error!!!\r\n");
		delay_ms(500);
		i++;
		if(i>4) {
			printf("Mode Set Error ,break\r\n");
			break;
		}
	}
	if(i>3){
		printf("Mode Set Error,over\r\n");
	}
	else {
		printf("Mode Set OK\r\n");
		vl53l0x_start_single_test(&vl53l0x_dev,&vl53l0x_data,D_x);
	}
}
void VL53L0X_calibration(void){
	VL53L0X_Error status = VL53L0X_ERROR_NONE;
	status = vl53l0x_adjust(&vl53l0x_dev);//进入校准
	if(status!=VL53L0X_ERROR_NONE)//校准失败
	{
		printf("calibration error！！！！\r\n");
	}
}

#endif 
