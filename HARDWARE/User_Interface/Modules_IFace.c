#include "main.h"
#include "Modules_IFace.h"
#include "mpu6050.h"

//界面
extern volatile unsigned char InterFace_Flag;
extern volatile unsigned char InterFace_Two_Flag;

#if MPU6050==1

void MPU6050_IFace(unsigned char show){
    static unsigned char MPU6050_Spam_Flag=0;
    float pitch,roll,yaw; 		    //欧拉角
    short aacx,aacy,aacz;		    //加速度传感器原始数据
    // short gyrox,gyroy,gyroz;	    //陀螺仪原始数据
    short temp;					    //温度
    char LCD_Debug_buf[20];
    if(MPU6050_Spam_Flag==1||show==1){
        LCD_Fill(0,0,240,240,BLACK);
        LCDDebug(10,0,"Press KEY3 to return");
        LCDDebug(10,16,"MPU6050:SDA-PB12,SCL-PB11,AOD-PB10,3.3V");
    }
    if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0){
			temp=MPU_Get_Temperature();	//得到温度值
			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
			if(temp<0){
				temp=-temp;		//转为正数
				sprintf(LCD_Debug_buf,"temp:-%d.%dC",temp/100,temp%10);
			}else {
				sprintf(LCD_Debug_buf,"temp:%d.%dC",temp/100,temp%10);
			}
			LCDDebug(18,48,LCD_Debug_buf);
			temp=pitch*10;
			if(temp<0){
				temp=-temp;		//转为正数
				sprintf(LCD_Debug_buf,"picht:-%d.%d",temp/10,temp%10);
			}else sprintf(LCD_Debug_buf,"picht:%d.%d",temp/10,temp%10);		//去掉负号
			LCDDebug(18,48+16,LCD_Debug_buf);

			temp=roll*10;
			if(temp<0){
				temp=-temp;		//转为正数
				sprintf(LCD_Debug_buf,"roll:-%d.%d",temp/10,temp%10);
			}else sprintf(LCD_Debug_buf,"roll:%d.%d",temp/10,temp%10);		//去掉负号
			LCDDebug(18,48+(16*2),LCD_Debug_buf);

			temp=yaw*10;
			if(temp<0){
				temp=-temp;		//转为正数
				sprintf(LCD_Debug_buf,"yaw:-%d.%d",temp/10,temp%10);
			}else sprintf(LCD_Debug_buf,"yaw:%d.%d",temp/10,temp%10);		//去掉负号
			LCDDebug(18,48+(16*3),LCD_Debug_buf);
    }
    switch(KEY_Flag){
			case 1:{
				KEY_Flag=0;
				GPIOA->ODR^=GPIO_Pin_2;
			}
			break;
			case 2:{
				KEY_Flag=0;
				GPIOA->ODR^=GPIO_Pin_2;
			}
			break;
			case 3:{
				KEY_Flag=0;
				InterFace_Two_Flag=0;
				InterFace_Flag=3;
				User_Modules_InterFace(0);
			}
			break;
			default:break;
	}
}

#endif

#if HC_SR04==1

void HC_SR04_IFace(unsigned char show){
    static unsigned char HC_SR04_Spam_Flag=0;
    char LCD_Debug_buf[20];
    if(HC_SR04_Spam_Flag==1||show==1){
        LCD_Fill(0,0,240,240,BLACK);
        LCDDebug(10,0,"Press KEY3 to return");
        LCDDebug(10,16,"HC_SR04:ECHO-PE11,TRIG-PE10");
    }
    if(HC_SR04_Error_Flag==0){
        sprintf(LCD_Debug_buf,"Distance:%4dmm",Get_Distance());
        LCDDebug(10,48,LCD_Debug_buf);
    }
    else {
        LCDDebug(10,48,"HC_SR04 is error!!!!!");
    }
    switch(KEY_Flag){
        case 1:{
            KEY_Flag=0;
            GPIOA->ODR^=GPIO_Pin_2;
        }
        break;
        case 2:{
            KEY_Flag=0;
            GPIOA->ODR^=GPIO_Pin_2;
        }
        break;
        case 3:{
            KEY_Flag=0;
            InterFace_Two_Flag=0;
            InterFace_Flag=3;
            User_Modules_InterFace(0);
        }
        break;
				default:break;
    }
}
#endif

#if VL53L0X==1
unsigned char Menu_VL53L0X_Num=7;
directory VL53L0X_directory[]={		//模块显示菜单
	{1,18,120,"calibration",WHITE,BLACK},
	{2,18,120+16,"general/interrupt",WHITE,BLACK},
    {3,18,120+(16*2),"Default_Mode",WHITE,BLACK},
	{4,18,120+(16*3),"HIGH_ACCURACY",WHITE,BLACK},
	{5,18,120+(16*4),"LONG_RANGE",WHITE,BLACK},
	{6,18,120+(16*5),"HIGH_SPEED",WHITE,BLACK},
	{7,18,120+(16*6),"return",WHITE,BLACK}
};
void VL53L0X_IFace(unsigned char show){
    static unsigned char VL53L0X_Spam_Flag=0;
    static unsigned char IFace_VL53L0X_Cut_Falg=0;
    static unsigned char VL53L0X_Calibration_Run_Flag=0;
    char LCD_Debug_buf[20];
    if(VL53L0X_Spam_Flag==1||show==1){
        LCD_Fill(0,0,240,240,BLACK);
        Show_Menu_All(VL53L0X_directory,Menu_VL53L0X_Num,IFace_VL53L0X_Cut_Falg);
		VL53L0X_Spam_Flag=0;
        LCDDebug(10,16,"VL53L0X:GPIO1-PD1,XSHUT-PD0,SCL-PD3,SDA-PD4,5V");
       
    } 
    if(VL53L0X_Calibration_Run_Flag==1){
        VL53L0X_calibration();      //校准
        VL53L0X_Calibration_Run_Flag=0;
    }
    // VL53L0X_Get_Common(HIGH_SPEED);
    // sprintf(LCD_Debug_buf,"Distance is %4imm",Distance_data);
    // LCDDebug(10,48,LCD_Debug_buf);
    // delay_ms(30);

    switch(KEY_Flag){
        case 1:{
            KEY_Flag=0;
            GPIOA->ODR^=GPIO_Pin_2;
            VL53L0X_Spam_Flag=1;
			if(IFace_VL53L0X_Cut_Falg==0){
				IFace_VL53L0X_Cut_Falg=0;
			}
			else if(IFace_VL53L0X_Cut_Falg!=0&&IFace_VL53L0X_Cut_Falg>0){
				IFace_VL53L0X_Cut_Falg-=1;
			}
        }
        break;
        case 2:{
					KEY_Flag=0;
					GPIOA->ODR^=GPIO_Pin_2;
					VL53L0X_Spam_Flag=1;
					if(IFace_VL53L0X_Cut_Falg<=Menu_VL53L0X_Num){
						IFace_VL53L0X_Cut_Falg+=1;
					}
        }
        break;
        case 3:{
            KEY_Flag=0;
            GPIOA->ODR^=GPIO_Pin_2;
            switch(IFace_VL53L0X_Cut_Falg){
                case 1:{
                    VL53L0X_Calibration_Run_Flag=1;
                }
                break;
                case 2:{

                }
                break;
                case 3:{

                }
                break;
                case 4:{

                }
                break;
                case 5:{

                }
                break;
                case 6:{

                }
                break;
                case 7:{
                    InterFace_Two_Flag=0;
                    InterFace_Flag=3;
                    User_Modules_InterFace(1);
                }
                break;
            }
        }
        break;
				default:break;
    }
}
#endif

#if BH1750==1

void BH1750_IFace(unsigned char show){
    static unsigned char BH1750_Spam_Flag=0;
    char LCD_Debug_buf[20];
    if(BH1750_Spam_Flag==1||show==1){
        LCD_Fill(0,0,240,240,BLACK);
        LCDDebug(16,0,"Press KEY3 to return");
        LCDDebug(16,16,"BH1750:SDA-PC5,SCL-PC4,5V");
    }
    sprintf(LCD_Debug_buf,"Light:%  9.2fNit",Get_BH1750());
    LCDDebug(16,48,LCD_Debug_buf);
    delay_ms(200);
    switch(KEY_Flag){
        case 1:{
            KEY_Flag=0;
            GPIOA->ODR^=GPIO_Pin_2;
        }
        break;
        case 2:{
            KEY_Flag=0;
            GPIOA->ODR^=GPIO_Pin_2;
        }
        break;
        case 3:{
            KEY_Flag=0;
            InterFace_Two_Flag=0;
            InterFace_Flag=3;
            User_Modules_InterFace(0);
        }
        break;
				default:break;
    }
}

#endif
