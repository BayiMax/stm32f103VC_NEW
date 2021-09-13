#include "SD_IFace.h"
#include "main.h"

#if SD==1

//界面
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
/*SD卡基础信息
//        switch(SDCardInfo.CardType){
//            case SDIO_STD_CAPACITY_SD_CARD_V1_1:LCD_ShowString(0,16,"SD is SDSC V1.1",WHITE,BLACK,16,0);break;
//            case SDIO_STD_CAPACITY_SD_CARD_V2_0:LCD_ShowString(0,16,"SD is SDSC V2.0",WHITE,BLACK,16,0);break;
//            case SDIO_HIGH_CAPACITY_SD_CARD:LCD_ShowString(0,16,"SD is SDSC V2.0",WHITE,BLACK,16,0);
//            case SDIO_MULTIMEDIA_CARD:LCD_ShowString(0,16,"SD is MMC Card",WHITE,BLACK,16,0);
//        }
//        sprintf(LCD_Debug_buf,"SD ID:%d;SD RCA:%d;SD RL:%d MB;SD BLOCK:%d" ,SDCardInfo.SD_cid.ManufacturerID,SDCardInfo.RCA,\
//        (u32)(SDCardInfo.CardCapacity>>20),SDCardInfo.CardBlockSize);
*/
			while(exf_getfree("0",&total,&free))	//得到SD卡的总容量和剩余容量
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

/*汉字显示实验*/
void SD_TXET_IFace(unsigned char show){
	static unsigned char SD_TXET_Spam_Flag=0;
	unsigned char Size;
	Size=16;
	if(SD_TXET_Spam_Flag==1||show==1){
		LCD_Fill(0,0,240,240,BLACK);
		for(unsigned char i=0;i<10;i++){
			Show_Str(0,Size*i,200,Size,"我喜欢张海雲q(≧▽≦q)",Size,0);
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
	unsigned char PICTURE_errorflag=0;		//图片文件错误标志位

	unsigned char Picture_Num_error=0;

	static unsigned char SD_Picture_Spam_Flag=0;

	DIR picdir;	 		//图片目录
	FILINFO picfileinfo;//文件信息
	u8 *fn;   			//长文件名
	u8 *pname;			//带路径的文件名
	u16 totpicnum; 		//图片文件总数
	u16 curindex;		//图片当前索引
	u8 pause;			//暂停标记
	u16 temp;
	u16 *picindextbl;	//图片索引表

	static u8 res;
	if(SD_Picture_Spam_Flag==1||show==1){
		LCD_Fill(0,0,240,240,BLACK);
		printf("清屏\r\n");
		
		while(f_opendir(&picdir,"0:/PICTURE")){		//打开图片文件夹
			Show_Str(0,0,240,240,"PICTURE文件错误",16,0);
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
			while(totpicnum==NULL){		//没有图片
				Show_Str(0,0,240,16,"没有图片文件!",16,0);
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
				picfileinfo.lfsize=_MAX_LFN*2+1;		//长文件名最大长度
				picfileinfo.lfname=mymalloc(SRAMIN,picfileinfo.lfsize);	//为长文件缓存区分配内存
				pname=mymalloc(SRAMIN,picfileinfo.lfsize);				//为带路径的文件名分配内存
				picindextbl=mymalloc(SRAMIN,2*totpicnum);				//申请2*totpicnum个字节的内存,用于存放图片索引
				while(picfileinfo.lfname==NULL||pname==NULL||picindextbl==NULL)//内存分配出错
				{
					Show_Str(0,16*1,240,16,"内存分配失败!",16,0);
					delay_ms(250);
					Show_Str(0,16*1,240,16,"             ",16,0);
					delay_ms(250);
				}
				res=f_opendir(&picdir,"0:/PICTURE"); //打开目录
				Show_Str(0,16*2,240,16,"正在检查图片",16,0);
				if(res==FR_OK)
				{
					curindex=0;		//当前索引为0
					while(1)		//全部查询一遍
					{
						temp=picdir.index;		//记录当前index
						res=f_readdir(&picdir,&picfileinfo);		//读取目录下的一个文件
						if(res!=FR_OK||picfileinfo.fname[0]==0)break;		//错误了/到末尾了,退出		  
						fn=(u8*)(*picfileinfo.lfname?picfileinfo.lfname:picfileinfo.fname);
						res=f_typetell(fn);
						if((res&0XF0)==0X50)		//取高四位,看看是不是图片文件	
						{
							picindextbl[curindex]=temp;		//记录索引
							curindex++;
						}
					}
				}
				Show_Str(0,16*3,240,240,"开始显示...",16,0); 
				delay_ms(1500);
				piclib_init();		//初始化画图	   	   
				curindex=0;		//从0开始显示
				res=f_opendir(&picdir,(const TCHAR*)"0:/PICTURE");		//打开目录

				while(res==FR_OK)		//打开成功
				{
					dir_sdi(&picdir,picindextbl[curindex]);		//改变当前目录索引	   
					res=f_readdir(&picdir,&picfileinfo);		//读取目录下的一个文件
					if(res!=FR_OK||picfileinfo.fname[0]==0){		//错误了/到末尾了,退出	
						Show_Str(0,0,240,16,"到头了，没得了",16,0);
					}
					else{
						fn=(u8*)(*picfileinfo.lfname?picfileinfo.lfname:picfileinfo.fname);			 
						strcpy((char*)pname,"0:/PICTURE/");		//复制路径(目录)
						strcat((char*)pname,(const char*)fn);		//将文件名接在后面
						LCD_Fill(0,0,240,16,BLACK);		//显示图片名字
						Show_Str(0,0,240,16,pname,16,0);		//显示图片名字
						LCD_Fill(0,16,240,240,BLACK);
						ai_load_picfile(pname,0,16,240,224,1);		//显示图片
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
							if(curindex>=totpicnum)curindex=0;//到末尾的时候,自动从头开始
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
	myfree(SRAMIN,picfileinfo.lfname);		//释放内存
	myfree(SRAMIN,pname);		//释放内存
	myfree(SRAMIN,picindextbl);		//释放内存
	InterFace_Two_Flag=0;
	InterFace_Flag=1;
	SD_Casd_InterFace(1);
}

#endif


