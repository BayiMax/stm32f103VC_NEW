#include "main.h"

#if W25Q64_Compile==1
//  W25Q64���Ա���
// const u8 TEXT_Buffer[]={"ELITE STM32 SPI TEST"};
// #define SIZE(T_Size) sizeof(T_Size)
// u8 REXT_Buffer[SIZE(TEXT_Buffer)]={"bbxre stm32 spi reer"};
// u8 *buf=0;
#endif /**/
//����
extern volatile unsigned char InterFace_Flag;
extern volatile unsigned char InterFace_Two_Flag;

/*SD��*/
#if SD ==1
unsigned int sd_size;
#endif

#if FATFS_EN==1
/*FATFS*/
unsigned char res=0;
#endif 
#if TEXT==1
/*TXET*/
unsigned char TXET_key=0;
#endif

/*��ʼ����ʱ��־λ*/
unsigned char Timeout_Flag=0;
/**/
/*���ݴ�ӡ����*/
char LCD_Debug_buf[40];
/**/
int main(void)
{
	delay_init();
	uart_init(115200);
	Key_IO_Init();
	RGB_IO_Init();
	LCD_Init();
	MCU_Temperature_ADC_Init();
	delay_ms(50);
	LCD_Fill(0,0,240,240,BLACK);
	printf("ready..\r\n");

	#if IP5328P==1
	Power_Get_Init();
	#endif 

	#if MPU6050==1
	Timeout_Flag=0;
	while(mpu_dmp_init()){
		LCDDebug(0,0,"MPU6050 Error");
		delay_ms(200);
		LCDDebug(0,0,"             ");
		delay_ms(200);
		Timeout_Flag++;
		if(Timeout_Flag>5){
		break;
		}
	}
	if(Timeout_Flag<4){
		LCDDebug(0,0,"MPU6050 is Ok");
		delay_ms(1000);
	}
	#endif 
	#if VL53L0X==1

	Vl53L0X_Init();

	#endif
	#if HC_SR04==1
	Timeout_Flag=0;
	while (HC_SR04_Init())		//��������ʼ��
	{
		LCDDebug(0,0,"HC_SR04 is ERror             ");
		delay_ms(200);
		LCDDebug(0,0,"                             ");
		delay_ms(200);
		Timeout_Flag++;
		if(Timeout_Flag>5){
		break;
		}
	}
	if(Timeout_Flag<4){
		LCDDebug(0,0,"HC_SR04 is Ok");
		delay_ms(1000);
	}
	#endif

	#if BH1750==1
	BH1750_Init();
	#endif 

	#if W25Q64_Compile == 1 
	W25Qxx_SPIAndIO_Init();
	/*W25Q64��ʼ�����*/
	while(1){
		if(W25QXX_ReadID()==W25Q64){
			break;
		}
		else {
			sprintf(LCD_Debug_buf,"ID :0x%X           ",W25QXX_ReadID());
			LCDDebug(0,32,LCD_Debug_buf);
			LCDDebug(0,16,"W25Qxx Error       ");
			Timeout_Flag++;
			delay_ms(200);
			if(KEY_Flag==1){
				Timeout_Flag=0;
				KEY_Flag=0;
				break;
			}
			if(Timeout_Flag>6){
				Timeout_Flag=0;
				break;
			}
		}
	}
	#endif

	#if malloc_Compile==1
	my_mem_init(SRAMIN);		//��ʼ���ڲ��ڴ��
	#endif

	#if FATFS_EN==1
	if(exfuns_init()!=0){		//Ϊfatfs��ر��������ڴ�
		LCDDebug(0,16*5,"Memory request failed");
	}
	f_mount(fs[0],"0:",1);		//����SD��
 	f_mount(fs[1],"1:",1);		//����FLASH

/*
//	if(res==0x0D){		//FLASH���̣�FAT�ļ�ϵͳ�������¸�ʽ��FLASH
//		LCDDebug(0,16*0,"FAT system error   ");
//		LCDDebug(0,16*1,"Format Fllsh       ");
//
//		res=f_mkfs("1",1,4096);		//��ʽ��FLASH,1,�̷�;1,����Ҫ������,8������Ϊ1����
//		if(res==0){
//			f_setlabel((const TCHAR *)"1:BBX");; //����Flash���̵�����Ϊ��BBX
//			LCDDebug(0,16*2,"Format to BBX      ");
//			LCDDebug(0,16*3,"Format Fllsh OK    ");
//		}else {
//			LCDDebug(0,16*2,"Format Fllsh error ");
//		}
//		delay_ms(1000);
//	}
*/
	#endif
	LCD_ShowString(0,0,"W:240;H:240",GBLUE,WHITE,16,1);

	#if SD==1
/*SD����ʼ�����*/
	while(SD_Init()){
		LCDDebug(0,16,"Sd is Error       ");
		delay_ms(250);
		LCDDebug(0,0,"                  ");
		delay_ms(250);
		Timeout_Flag++;
		if(KEY_Flag==1){
			Timeout_Flag=0;
			KEY_Flag=0;
			break;
		}
		if(Timeout_Flag>6){
			Timeout_Flag=0;
			break;
		}
	}
	#endif
/**/
	#if TEXT==1
/*//����ֿ�*/
	while(font_init()){
		TXET_key=update_font(20,110,16,"0:");//�����ֿ�
		while(TXET_key){
			LCDDebug(0,0,"Font Update Failed!");		//����ʧ��
		}
		LCDDebug(0,0,"Font Update Success!");
	}
	delay_ms(1000);
	#endif 
/**/
	LCD_Fill(0,0,240,240,BLACK);
	Load_Animation();
	User_Main_InterFace(1);
 	while(1)
	{
		switch(InterFace_Flag)
		{
			case 0:User_Main_InterFace(0);break;
			case 1:{
				#if SD==1
				switch(InterFace_Two_Flag){
					case 0:SD_Casd_InterFace(0);break;
					case 1:SD_IFace_information(0);break;
					case 2:break;
					case 3:SD_TXET_IFace(0);break;
					case 4:SD_Picture_IFace(0);break;
					default:break;
				}
				#endif
			}
			break;
			case 3:{
				switch(InterFace_Two_Flag){
					case 0:User_Modules_InterFace(0);break;
					case 1:break;
					
					case 3:
						#if HC_SR04==1
							HC_SR04_IFace(0);
						#endif
						break;
					case 4:
						#if MPU6050==1
						MPU6050_IFace(0);
						#endif
						break;
					
					case 5:
						#if VL53L0X==1
						VL53L0X_IFace(0);
						#endif
						break;
					
					case 6:
						#if BH1750==1
						BH1750_IFace(0);
						#endif 
						break;
					default:break;
				}
			}
			break;
			default:break;
		}
	}
}

