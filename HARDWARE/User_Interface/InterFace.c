#include "InterFace.h"
#include "main.h"

volatile unsigned char InterFace_Flag=0;
volatile unsigned char InterFace_Two_Flag=0;

char Debug_buf[40];

/*********************************************/

void Show_Menu_One(directory *directory_DP,unsigned char Select_Frame){
	if(Select_Frame==0){
		LCD_ShowString(directory_DP->x,directory_DP->y,directory_DP->p,directory_DP->FC,directory_DP->BC,16,0);
	}
	else if(Select_Frame==1){
		LCD_ShowString(directory_DP->x,directory_DP->y,directory_DP->p,directory_DP->BC,directory_DP->FC,16,0);
	}
}

void Show_Menu_All(directory *directory_DP,unsigned char num,unsigned char Select_Frame){
	unsigned char i;
	for(i=0;i<num;i++){
		if(i+1==Select_Frame&&Select_Frame!=0){
			Show_Menu_One(&directory_DP[i],1);
		}
		else{
			Show_Menu_One(&directory_DP[i],0);
		}
	}
}

/*首页菜单定义*/
unsigned char Menu_Main_Num=3;
directory Main_directory[]={		//主页面显示菜单
	{0,18,20,"SD",WHITE,BLACK},
	{1,18,20+16,"W25Q64",WHITE,BLACK},
	{2,18,20+(16*2),"Modules",WHITE,BLACK},
};
/*********************************************/
/*开机动画*/
void Load_Animation(void){
	unsigned char JDT_H=10;
	unsigned char JDT_BJ=0;
	LCD_Fill(0,0,240,240,BLACK);
	LCD_ShowString(92,112,"LOADing",WHITE,BLACK,16,0);
	LCD_DrawRectangle(60,128,180,128+JDT_H,WHITE);
	do{
		LCD_DrawLine(60+JDT_BJ,128,60+JDT_BJ,128+JDT_H,WHITE);
		delay_ms(25);
		JDT_BJ++;
	}while(JDT_BJ<120);
	LCD_Fill(0,0,240,240,BLACK);
	Show_Menu_All(Main_directory,3,0);
}
/*首页面*/
void User_Main_InterFace(unsigned char show){
	static unsigned char IF_Main_Spam_Flag=0;
	static unsigned char IFace_Main_Cut_Falg=0;
	sprintf(Debug_buf,"T:%.2fC",((float)Get_Temprate())/100);
	// LCDDebug(0,0,Debug_buf);
	LCD_ShowString(0,0,Debug_buf,GBLUE,BLACK,16,0);
	#if IP5328P==1
	sprintf(Debug_buf,"V:%.2fV A:%.4fA",IP5328P_BatVoltage(),IP5328P_BatCurrent());
	// LCDDebug(8*9,0,Debug_buf);
	LCD_ShowString(8*9,0,Debug_buf,GBLUE,BLACK,16,0);
	#endif
	if(IF_Main_Spam_Flag==1||show==1){
		LCD_Fill(0,0,240,224,BLACK);
		Show_Menu_All(Main_directory,Menu_Main_Num,IFace_Main_Cut_Falg);
		IF_Main_Spam_Flag=0;
	}
	switch(KEY_Flag){
		case 1:{
			KEY_Flag=0;
			GPIOA->ODR^=GPIO_Pin_1;
			IF_Main_Spam_Flag=1;
			if(IFace_Main_Cut_Falg==0){
				IFace_Main_Cut_Falg=0;
			}
			else if(IFace_Main_Cut_Falg!=0&&IFace_Main_Cut_Falg>0){
				IFace_Main_Cut_Falg-=1;
			}
			break;
		}
		case 2:{
			KEY_Flag=0;
			GPIOA->ODR^=GPIO_Pin_1;
			IF_Main_Spam_Flag=1;
			if(IFace_Main_Cut_Falg<=Menu_Main_Num){
				IFace_Main_Cut_Falg+=1;
			}
			break;
		}
		case 3:{
			KEY_Flag=0;
			GPIOA->ODR^=GPIO_Pin_1;
			IF_Main_Spam_Flag=1;
			switch(IFace_Main_Cut_Falg){
				case 1:{
					#if SD==1
					InterFace_Flag=1;
					IFace_Main_Cut_Falg=0;
					SD_Casd_InterFace(1);
					#endif
				}
				break;
				case 3:{
					#if VL53L0X==1||MPU6050==1||BH1750==1||HC_SR04==1
					InterFace_Flag=3;
					IFace_Main_Cut_Falg=0;
					User_Modules_InterFace(1);
					#endif
				}
				break;
			}
			break;
		}
		default:;break;
	}
}

#if SD==1		//SD界面
/*SD卡菜单定义*/
unsigned char Menu_SD_Num=5;
//unsigned char Sd_Card_Directory_H=0;
directory SD_Casd_directory[]={		//SD显示菜单
	{0,18,20,"SD information",WHITE,BLACK},
	{1,18,20+16,"yuliou",WHITE,BLACK},
	{2,18,20+(16*2),"TXET",WHITE,BLACK},
	{3,18,20+(16*3),"picture",WHITE,BLACK},
	{4,18,20+(16*4),"return",WHITE,BLACK}
};
/*********************************************/
void SD_Casd_InterFace(unsigned char Show){
	static unsigned char IF_SD_Spam_Flag=0;
	static unsigned char IFace_SD_Cut_Falg=0;
	if(IF_SD_Spam_Flag==1||Show==1){
		LCD_Fill(0,0,240,240,BLACK);
		Show_Menu_All(SD_Casd_directory,Menu_SD_Num,IFace_SD_Cut_Falg);
		IF_SD_Spam_Flag=0;
	}
	switch(KEY_Flag){
		case 1:{
			KEY_Flag=0;
			GPIOA->ODR^=GPIO_Pin_1;
			IF_SD_Spam_Flag=1;
			if(IFace_SD_Cut_Falg==0){
				IFace_SD_Cut_Falg=0;
			}
			else if(IFace_SD_Cut_Falg!=0&&IFace_SD_Cut_Falg>0){
				IFace_SD_Cut_Falg-=1;
			}
		}
		break;
		case 2:{
			KEY_Flag=0;
			GPIOA->ODR^=GPIO_Pin_1;
			IF_SD_Spam_Flag=1;
			if(IFace_SD_Cut_Falg<=Menu_SD_Num){
				IFace_SD_Cut_Falg+=1;
			}
		}
		break;
		case 3:{
			KEY_Flag=0;
			GPIOA->ODR^=GPIO_Pin_1;
			switch(IFace_SD_Cut_Falg){
				case 1:{
					InterFace_Flag=1;
					InterFace_Two_Flag=1;
					SD_IFace_information(1);
				}
				break;
				case 2:{
//					InterFace_Flag=1;
//					InterFace_Two_Flag=2;
//					SD_Picture_IFace(1);
				}
				break;
				case 3:{
					InterFace_Flag=1;
					InterFace_Two_Flag=3;
					SD_TXET_IFace(1);
				}
				break;
				case 4:{
					InterFace_Flag=1;
					InterFace_Two_Flag=4;
					SD_Picture_IFace(1);
				}
				break;
				case 5:{
					InterFace_Flag=0;
					InterFace_Two_Flag=0;
					IFace_SD_Cut_Falg=0;
					User_Main_InterFace(1);
				}
				break;
				default:break;
			}
		}
		break;
		default:break;
	}
}
#endif

/**********************************************************/
#define Menu_Modules_Num 7
directory Modules_directory[]={		//模块显示菜单
	{1,18,20,"Relay",WHITE,BLACK},
	{2,18,20+16,"Bluetooth",WHITE,BLACK},
	{3,18,20+(16*2),"HC-SR04",WHITE,BLACK},
	{4,18,20+(16*3),"MPU6050",WHITE,BLACK},
	{5,18,20+(16*4),"VL53L0X",WHITE,BLACK},
	{6,18,20+(16*5),"BH1750",WHITE,BLACK},
	{Menu_Modules_Num,18,20+(16*(Menu_Modules_Num-1)),"return",WHITE,BLACK}
};

/*模块界面*/
void User_Modules_InterFace(unsigned char show){
	static unsigned char IF_Modules_Spam_Flag=0;
	static unsigned char IFace_Modules_Cut_Falg=0;
	if(IF_Modules_Spam_Flag==1||show==1){
		LCD_Fill(0,0,240,240,BLACK);
		Show_Menu_All(Modules_directory,Menu_Modules_Num,IFace_Modules_Cut_Falg);
		IF_Modules_Spam_Flag=0;
	}
	switch(KEY_Flag){
		case 1:{
			KEY_Flag=0;
			GPIOA->ODR^=GPIO_Pin_1;
			IF_Modules_Spam_Flag=1;
			if(IFace_Modules_Cut_Falg==0){
				IFace_Modules_Cut_Falg=0;
			}
			else if(IFace_Modules_Cut_Falg!=0&&IFace_Modules_Cut_Falg>0){
				IFace_Modules_Cut_Falg-=1;
			}
		}
		break;
		case 2:{
			KEY_Flag=0;
			GPIOA->ODR^=GPIO_Pin_1;
			IF_Modules_Spam_Flag=1;
			if(IFace_Modules_Cut_Falg<=Menu_Modules_Num){
				IFace_Modules_Cut_Falg+=1;
			}
		}
		break;
		case 3:{
			KEY_Flag=0;
			GPIOA->ODR^=GPIO_Pin_1;
			IF_Modules_Spam_Flag=1;
			switch(IFace_Modules_Cut_Falg){
				case 1:{
				}
				break;
				case 2:{
					
				}
				break;
				#if HC_SR04==1
				case 3:{
					InterFace_Flag=3;
					InterFace_Two_Flag=3;
					IFace_Modules_Cut_Falg=0;
					HC_SR04_IFace(1);
				}
				break;
				#endif 
				#if MPU6050==1
				case 4:{
					InterFace_Flag=3;
					InterFace_Two_Flag=4;
					IFace_Modules_Cut_Falg=0;
					MPU6050_IFace(1);
				}
				break;
				#endif
				#if VL53L0X==1
				case 5:{
					InterFace_Flag=3;
					InterFace_Two_Flag=5;
					IFace_Modules_Cut_Falg=0;
					VL53L0X_IFace(1);
				}
				break;
				#endif
				#if BH1750==1
				case 6:{
					InterFace_Flag=3;
					InterFace_Two_Flag=6;
					IFace_Modules_Cut_Falg=0;
					BH1750_IFace(1);
				}
				break;
				#endif
				case Menu_Modules_Num:{
					InterFace_Flag=0;
					InterFace_Two_Flag=0;
					IFace_Modules_Cut_Falg=0;
					User_Main_InterFace(1);
				}
				break;
			}
		}
		break;
		default:break;
	}
}

