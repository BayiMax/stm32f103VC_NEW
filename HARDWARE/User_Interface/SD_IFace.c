#include "SD_IFace.h"
#include "main.h"

#if SD==1

//����
extern volatile unsigned char InterFace_Flag;
extern volatile unsigned char InterFace_Two_Flag;

void SD_IFace_information(unsigned char show){
    static unsigned char SD_Information_Spam_Flag=0;
    char LCD_Debug_buf[60];
	 	u32 total,free;
    GPIOA->ODR^=GPIO_Pin_2;
    if(SD_Information_Spam_Flag==1||show==1){
			LCD_Fill(0,0,240,240,BLACK);
			LCDDebug(10,0,"Press KEY3 to return");
/*SD��������Ϣ
//        switch(SDCardInfo.CardType){
//            case SDIO_STD_CAPACITY_SD_CARD_V1_1:LCD_ShowString(0,16,"SD is SDSC V1.1",WHITE,BLACK,16,0);break;
//            case SDIO_STD_CAPACITY_SD_CARD_V2_0:LCD_ShowString(0,16,"SD is SDSC V2.0",WHITE,BLACK,16,0);break;
//            case SDIO_HIGH_CAPACITY_SD_CARD:LCD_ShowString(0,16,"SD is SDSC V2.0",WHITE,BLACK,16,0);
//            case SDIO_MULTIMEDIA_CARD:LCD_ShowString(0,16,"SD is MMC Card",WHITE,BLACK,16,0);
//        }
//        sprintf(LCD_Debug_buf,"SD ID:%d;SD RCA:%d;SD RL:%d MB;SD BLOCK:%d" ,SDCardInfo.SD_cid.ManufacturerID,SDCardInfo.RCA,\
//        (u32)(SDCardInfo.CardCapacity>>20),SDCardInfo.CardBlockSize);
*/
			while(exf_getfree("0",&total,&free))	//�õ�SD������������ʣ������
			{
				LCDDebug(0,16*1,"SD Card FAT error");
				delay_ms(250);
				LCDDebug(0,16*1,"                 ");
				GPIOA->ODR^=GPIO_Pin_1;
				delay_ms(250);
				LCDDebug(0,16*2,"Press KEY1 to break");
				
				if(KEY_Flag==1){
					KEY_Flag=0;
					break;
				}
			}
			sprintf(LCD_Debug_buf,"SD Total Size:%dMB,SD Free Size:%dMB",total>>10,free>>10);
			LCD_ShowString(0,16*2,LCD_Debug_buf,WHITE,BLACK,16,0);
    }
    GPIOA->ODR^=GPIO_Pin_3;
    switch(KEY_Flag){
        case 1:{
            KEY_Flag=0;
            GPIOA->ODR^=GPIO_Pin_2;
            break;
        }
        case 2:{
            KEY_Flag=0;
            GPIOA->ODR^=GPIO_Pin_2;
            break;
        }
        case 3:{
            KEY_Flag=0;
            InterFace_Two_Flag=0;
            InterFace_Flag=1;
            SD_Casd_InterFace(1);
            break;
        }
    }
}

/*������ʾʵ��*/
void SD_TXET_IFace(unsigned char show){
	static unsigned char SD_TXET_Spam_Flag=0;
	unsigned char Size;
	Size=16;
	if(SD_TXET_Spam_Flag==1||show==1){
		LCD_Fill(0,0,240,240,BLACK);
		for(unsigned char i=0;i<10;i++){
			Show_Str(0,Size*i,200,Size,"��ϲ���ź��q(�R���Qq)",Size,0);
		}
	}
	GPIOA->ODR^=GPIO_Pin_3;
	switch(KEY_Flag){
		case 1:{
			KEY_Flag=0;
			GPIOA->ODR^=GPIO_Pin_2;
			break;
		}
		case 2:{
			KEY_Flag=0;
			GPIOA->ODR^=GPIO_Pin_2;
			break;
		}
		case 3:{
			KEY_Flag=0;
			InterFace_Two_Flag=0;
			InterFace_Flag=1;
			SD_Casd_InterFace(1);
			break;
		}
	}
}

const char geometry_Picture[10]={"0:/PICTURE"};

// static unsigned char Picture_Init_Flag=0;

void SD_Picture_IFace(unsigned char show){
	unsigned char Time_Out_Flag=0;
	unsigned char PICTURE_errorflag=0;		//ͼƬ�ļ������־λ

	unsigned char Picture_Num_error=0;

	static unsigned char SD_Picture_Spam_Flag=0;

	DIR picdir;	 		//ͼƬĿ¼
	FILINFO picfileinfo;//�ļ���Ϣ
	u8 *fn;   			//���ļ���
	u8 *pname;			//��·�����ļ���
	u16 totpicnum; 		//ͼƬ�ļ�����
	u16 curindex;		//ͼƬ��ǰ����
	u8 pause;			//��ͣ���
	u16 temp;
	u16 *picindextbl;	//ͼƬ������

	static u8 res;
	if(SD_Picture_Spam_Flag==1||show==1){
		LCD_Fill(0,0,240,240,BLACK);
		printf("����\r\n");
		
		while(f_opendir(&picdir,"0:/PICTURE")){		//��ͼƬ�ļ���
			Show_Str(0,0,240,240,"PICTURE�ļ�����",16,0);
			delay_ms(250);
			Show_Str(0,0,240,240,"               ",16,0);
			delay_ms(250);
			Time_Out_Flag++;
			if(Time_Out_Flag>6){
				PICTURE_errorflag=0xff;
				break;
			}
		}
		Time_Out_Flag=0;
		if(PICTURE_errorflag!=0xff){
			totpicnum=pic_get_tnum("0:/PICTURE");
			while(totpicnum==NULL){		//û��ͼƬ
				Show_Str(0,0,240,16,"û��ͼƬ�ļ�!",16,0);
				delay_ms(250);
				Show_Str(0,0,240,16,"             ",16,0);
				delay_ms(250);
				Time_Out_Flag++;
				if(Time_Out_Flag>6){
					Picture_Num_error=0xff;
					break;
				}
			}
			if(Picture_Num_error!=0xff){
				picfileinfo.lfsize=_MAX_LFN*2+1;		//���ļ�����󳤶�
				picfileinfo.lfname=mymalloc(SRAMIN,picfileinfo.lfsize);	//Ϊ���ļ������������ڴ�
				pname=mymalloc(SRAMIN,picfileinfo.lfsize);				//Ϊ��·�����ļ��������ڴ�
				picindextbl=mymalloc(SRAMIN,2*totpicnum);				//����2*totpicnum���ֽڵ��ڴ�,���ڴ��ͼƬ����
				while(picfileinfo.lfname==NULL||pname==NULL||picindextbl==NULL)//�ڴ�������
				{
					Show_Str(0,16*1,240,16,"�ڴ����ʧ��!",16,0);
					delay_ms(250);
					Show_Str(0,16*1,240,16,"             ",16,0);
					delay_ms(250);
				}
				res=f_opendir(&picdir,"0:/PICTURE"); //��Ŀ¼
				Show_Str(0,16*2,240,16,"���ڼ��ͼƬ",16,0);
				if(res==FR_OK)
				{
					curindex=0;		//��ǰ����Ϊ0
					while(1)		//ȫ����ѯһ��
					{
						temp=picdir.index;		//��¼��ǰindex
						res=f_readdir(&picdir,&picfileinfo);		//��ȡĿ¼�µ�һ���ļ�
						if(res!=FR_OK||picfileinfo.fname[0]==0)break;		//������/��ĩβ��,�˳�		  
						fn=(u8*)(*picfileinfo.lfname?picfileinfo.lfname:picfileinfo.fname);
						res=f_typetell(fn);
						if((res&0XF0)==0X50)		//ȡ����λ,�����ǲ���ͼƬ�ļ�	
						{
							picindextbl[curindex]=temp;		//��¼����
							curindex++;
						}
					}
				}
				Show_Str(0,16*3,240,240,"��ʼ��ʾ...",16,0); 
				delay_ms(1500);
				piclib_init();		//��ʼ����ͼ	   	   
				curindex=0;		//��0��ʼ��ʾ
				res=f_opendir(&picdir,(const TCHAR*)"0:/PICTURE");		//��Ŀ¼

				while(res==FR_OK)		//�򿪳ɹ�
				{
					dir_sdi(&picdir,picindextbl[curindex]);		//�ı䵱ǰĿ¼����	   
					res=f_readdir(&picdir,&picfileinfo);		//��ȡĿ¼�µ�һ���ļ�
					if(res!=FR_OK||picfileinfo.fname[0]==0){		//������/��ĩβ��,�˳�	
						Show_Str(0,0,240,16,"��ͷ�ˣ�û����",16,0);
					}
					else{
						fn=(u8*)(*picfileinfo.lfname?picfileinfo.lfname:picfileinfo.fname);			 
						strcpy((char*)pname,"0:/PICTURE/");		//����·��(Ŀ¼)
						strcat((char*)pname,(const char*)fn);		//���ļ������ں���
						LCD_Fill(0,0,240,16,BLACK);		//��ʾͼƬ����
						Show_Str(0,0,240,16,pname,16,0);		//��ʾͼƬ����
						LCD_Fill(0,16,240,240,BLACK);
						ai_load_picfile(pname,0,16,240,224,1);		//��ʾͼƬ
					}
					while(1){
						if(KEY_Flag==1){
							KEY_Flag=0;
							GPIOA->ODR^=GPIO_Pin_2;
							if(curindex)curindex--;
							else curindex=totpicnum-1;
							break;
						}
						else if(KEY_Flag==2){
							KEY_Flag=0;
							GPIOA->ODR^=GPIO_Pin_2;
							curindex++;		   	
							if(curindex>=totpicnum)curindex=0;//��ĩβ��ʱ��,�Զ���ͷ��ʼ
							break;
						}
						else if(KEY_Flag==3){
							KEY_Flag=0;
							GPIOA->ODR^=GPIO_Pin_2;
							goto Picture_Over;
						}
					}
				}
			}
		}
	}
	Picture_Over:
	myfree(SRAMIN,picfileinfo.lfname);		//�ͷ��ڴ�
	myfree(SRAMIN,pname);		//�ͷ��ڴ�
	myfree(SRAMIN,picindextbl);		//�ͷ��ڴ�
	InterFace_Two_Flag=0;
	InterFace_Flag=1;
	SD_Casd_InterFace(1);
}

#endif


