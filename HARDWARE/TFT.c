#include "TFT.h"
#include "main.h"
#include "TFT_lcdfont.h"

DMA_InitTypeDef DMA_InitStructure;

u16 DMA1_MEM_LEN;//����DMAÿ�����ݴ��͵ĳ���

/*DMA1�ĸ�ͨ������
����Ĵ�����ʽ�ǹ̶���,���Ҫ���ݲ�ͬ��������޸�
�Ӵ洢��->����ģʽ/8λ���ݿ��/�洢������ģʽ
DMA_CHx:DMAͨ��CHx
cpar:�����ַ
cmar:�洢����ַ
cndtr:���ݴ����� */
static void MYDMA_Config(DMA_Channel_TypeDef* DMA_CHx,u32 cpar,u32 cmar,u16 cndtr){
 	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	//ʹ��DMA����

	DMA_DeInit(DMA_CHx);   //��DMA��ͨ��1�Ĵ�������Ϊȱʡֵ
	DMA1_MEM_LEN=cndtr;
	DMA_InitStructure.DMA_PeripheralBaseAddr = cpar;  //DMA����ADC����ַ
	DMA_InitStructure.DMA_MemoryBaseAddr = cmar;  //DMA�ڴ����ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;  //���ݴ��䷽�򣬴��ڴ��ȡ���͵�����
	DMA_InitStructure.DMA_BufferSize = cndtr;  //DMAͨ����DMA����Ĵ�С
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //�����ַ�Ĵ�������
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //�ڴ��ַ�Ĵ�������
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //���ݿ��Ϊ8λ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //���ݿ��Ϊ8λ
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  //��������������ģʽ
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; //DMAͨ�� xӵ�������ȼ�
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //DMAͨ��xû������Ϊ�ڴ浽�ڴ洫��
	DMA_Init(DMA_CHx, &DMA_InitStructure);  //����DMA_InitStruct��ָ���Ĳ�����ʼ��DMA��ͨ��USART1_Tx_DMA_Channel����ʶ�ļĴ���
}

static void MYDMA_Config1(DMA_Channel_TypeDef* DMA_CHx,u32 cpar,u32 cmar,u16 cndtr){
 	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	//ʹ��DMA����

  DMA_DeInit(DMA_CHx);   //��DMA��ͨ��1�Ĵ�������Ϊȱʡֵ
	DMA1_MEM_LEN=cndtr;
	DMA_InitStructure.DMA_PeripheralBaseAddr = cpar;  //DMA����ADC����ַ
	DMA_InitStructure.DMA_MemoryBaseAddr = cmar;  //DMA�ڴ����ַ
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;  //���ݴ��䷽�򣬴��ڴ��ȡ���͵�����
	DMA_InitStructure.DMA_BufferSize = cndtr;  //DMAͨ����DMA����Ĵ�С
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //�����ַ�Ĵ�������
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;  //�ڴ��ַ�Ĵ�������
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;  //���ݿ��Ϊ16λ
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; //���ݿ��Ϊ16λ
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  //��������������ģʽ
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; //DMAͨ�� xӵ�������ȼ�
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //DMAͨ��xû������Ϊ�ڴ浽�ڴ洫��
	DMA_Init(DMA_CHx, &DMA_InitStructure);  //����DMA_InitStruct��ָ���Ĳ�����ʼ��DMA��ͨ��USART1_Tx_DMA_Channel����ʶ�ļĴ���
}

//����һ��DMA����
static void MYDMA_Enable(DMA_Channel_TypeDef*DMA_CHx){
	DMA_Cmd(DMA_CHx, DISABLE );
 	DMA_SetCurrDataCounter(DMA1_Channel5,DMA1_MEM_LEN);
 	DMA_Cmd(DMA_CHx, ENABLE);
}

static void TFT_SPI_Init(void){
	GPIO_InitTypeDef TFT_SPI_IO_Init;
	SPI_InitTypeDef TFT_SPI_InitTypeDef;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);		//PORTBʱ��ʹ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD,ENABLE);		//PORTBʱ��ʹ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);		//SPI2ʱ��ʹ��

	TFT_SPI_IO_Init.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_15;
	TFT_SPI_IO_Init.GPIO_Mode = GPIO_Mode_AF_PP;  //PB13/15�����������
	TFT_SPI_IO_Init.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &TFT_SPI_IO_Init);//��ʼ��GPIOB
	GPIO_SetBits(GPIOB,GPIO_Pin_13|GPIO_Pin_15);  //PB13/15����

	TFT_SPI_InitTypeDef.SPI_Direction = SPI_Direction_1Line_Tx;//ֻ����
	TFT_SPI_InitTypeDef.SPI_Mode = SPI_Mode_Master;//��
	TFT_SPI_InitTypeDef.SPI_DataSize = SPI_DataSize_8b;//8λ֡
	TFT_SPI_InitTypeDef.SPI_CPOL = SPI_CPOL_High;//����ʱSCLK��
	TFT_SPI_InitTypeDef.SPI_CPHA = SPI_CPHA_2Edge;//����ͬ��ʱ�ӿյڶ���ʱ���ز���
	TFT_SPI_InitTypeDef.SPI_NSS = SPI_NSS_Soft;//NSS�ź��������
	TFT_SPI_InitTypeDef.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;//Ԥ��ƵֵΪ2
	TFT_SPI_InitTypeDef.SPI_FirstBit = SPI_FirstBit_MSB;//���ݸ�λ����
	TFT_SPI_InitTypeDef.SPI_CRCPolynomial = 7;//CRCֵ����Ķ���ʽ
	SPI_Init(SPI2,&TFT_SPI_InitTypeDef);
	SPI_Cmd(SPI2, ENABLE);

	delay_ms(1);

	TFT_SPI_IO_Init.GPIO_Pin = TFT_BLK | TFT_DC | TFT_RES;
	TFT_SPI_IO_Init.GPIO_Mode = GPIO_Mode_Out_PP;
	TFT_SPI_IO_Init.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &TFT_SPI_IO_Init);
	GPIO_SetBits(GPIOD,TFT_BLK | TFT_DC | TFT_RES);
}


/*SPI������*/
static void LCD_Writ_Bus(u8 dat){
  while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE) == RESET);//�����ձ�־λ
	SPI_I2S_SendData(SPI2,dat);
	delay_us(1);
}
/*TFTд8λ*/
static void LCD_WR_DATA8(u8 dat){
	LCD_Writ_Bus(dat);
}
/*TFTд16λ����*/
static void LCD_WR_DATA(u16 dat){
	LCD_Writ_Bus(dat>>8);
	LCD_Writ_Bus(dat);
}
/*TFTд����*/
static void LCD_WR_REG(u8 dat){
	LCD_DC_Clr();//д����
	LCD_Writ_Bus(dat);
	LCD_DC_Set();//д����
}
/*��ʼ�ͽ�����ַ
x1,x2 �����е���ʼ�ͽ�����ַ
y1,y2 �����е���ʼ�ͽ�����ַ
*/
void LCD_Address_Set(u16 x1,u16 y1,u16 x2,u16 y2){
	if(USE_HORIZONTAL==0){
		LCD_WR_REG(0x2a);//�е�ַ����
		LCD_WR_DATA(x1);
		LCD_WR_DATA(x2);
		LCD_WR_REG(0x2b);//�е�ַ����
		LCD_WR_DATA(y1);
		LCD_WR_DATA(y2);
		LCD_WR_REG(0x2c);//������д
	}
	else if(USE_HORIZONTAL==1){
		LCD_WR_REG(0x2a);//�е�ַ����
		LCD_WR_DATA(x1);
		LCD_WR_DATA(x2);
		LCD_WR_REG(0x2b);//�е�ַ����
		LCD_WR_DATA(y1+80);
		LCD_WR_DATA(y2+80);
		LCD_WR_REG(0x2c);//������д
	}
	else if(USE_HORIZONTAL==2){
		LCD_WR_REG(0x2a);//�е�ַ����
		LCD_WR_DATA(x1);
		LCD_WR_DATA(x2);
		LCD_WR_REG(0x2b);//�е�ַ����
		LCD_WR_DATA(y1);
		LCD_WR_DATA(y2);
		LCD_WR_REG(0x2c);//������д
	}
	else{
		LCD_WR_REG(0x2a);//�е�ַ����
		LCD_WR_DATA(x1+80);
		LCD_WR_DATA(x2+80);
		LCD_WR_REG(0x2b);//�е�ַ����
		LCD_WR_DATA(y1);
		LCD_WR_DATA(y2);
		LCD_WR_REG(0x2c);//������д
	}
}

void LCD_Init(void){
	TFT_SPI_Init();

	LCD_RES_Clr();//��λ
	delay_ms(100);
	LCD_RES_Set();
	delay_ms(100);

	LCD_BLK_Set();//�򿪱���
	delay_ms(100);

	//************* Start Initial Sequence **********//
	LCD_WR_REG(0x11);		//Sleep out
	delay_ms(120);		//Delay 120ms
	//************* Start Initial Sequence **********//
	LCD_WR_REG(0x36);
	if(USE_HORIZONTAL==0)LCD_WR_DATA8(0x00);
	else if(USE_HORIZONTAL==1)LCD_WR_DATA8(0xC0);
	else if(USE_HORIZONTAL==2)LCD_WR_DATA8(0x70);
	else LCD_WR_DATA8(0xA0);

	LCD_WR_REG(0x3A);
	LCD_WR_DATA8(0x05);

	LCD_WR_REG(0xB2);
	LCD_WR_DATA8(0x0C);
	LCD_WR_DATA8(0x0C);
	LCD_WR_DATA8(0x00);
	LCD_WR_DATA8(0x33);
	LCD_WR_DATA8(0x33);

	LCD_WR_REG(0xB7);
	LCD_WR_DATA8(0x35);

	LCD_WR_REG(0xBB);
	LCD_WR_DATA8(0x19);

	LCD_WR_REG(0xC0);
	LCD_WR_DATA8(0x2C);

	LCD_WR_REG(0xC2);
	LCD_WR_DATA8(0x01);

	LCD_WR_REG(0xC3);
	LCD_WR_DATA8(0x12);

	LCD_WR_REG(0xC4);
	LCD_WR_DATA8(0x20);

	LCD_WR_REG(0xC6);
	LCD_WR_DATA8(0x0F);

	LCD_WR_REG(0xD0);
	LCD_WR_DATA8(0xA4);
	LCD_WR_DATA8(0xA1);

	LCD_WR_REG(0xE0);
	LCD_WR_DATA8(0xD0);
	LCD_WR_DATA8(0x04);
	LCD_WR_DATA8(0x0D);
	LCD_WR_DATA8(0x11);
	LCD_WR_DATA8(0x13);
	LCD_WR_DATA8(0x2B);
	LCD_WR_DATA8(0x3F);
	LCD_WR_DATA8(0x54);
	LCD_WR_DATA8(0x4C);
	LCD_WR_DATA8(0x18);
	LCD_WR_DATA8(0x0D);
	LCD_WR_DATA8(0x0B);
	LCD_WR_DATA8(0x1F);
	LCD_WR_DATA8(0x23);

	LCD_WR_REG(0xE1);
	LCD_WR_DATA8(0xD0);
	LCD_WR_DATA8(0x04);
	LCD_WR_DATA8(0x0C);
	LCD_WR_DATA8(0x11);
	LCD_WR_DATA8(0x13);
	LCD_WR_DATA8(0x2C);
	LCD_WR_DATA8(0x3F);
	LCD_WR_DATA8(0x44);
	LCD_WR_DATA8(0x51);
	LCD_WR_DATA8(0x2F);
	LCD_WR_DATA8(0x1F);
	LCD_WR_DATA8(0x1F);
	LCD_WR_DATA8(0x20);
	LCD_WR_DATA8(0x23);
	LCD_WR_REG(0x21);
	LCD_WR_REG(0x29);
	delay_ms(100);
}

/*��ָ�����������ɫ
xsta,ysta   ��ʼ����
xend,yend   ��ֹ����
color       Ҫ������ɫ

***ˢ�����Ҫ1us��ʱʹ��spi8λ���ݴ���
*/
void LCD_Fill(u16 xsta,u16 ysta,u16 xend,u16 yend,u16 color){
	u16 color1[1],t=1;
	u32 num,num1;
	color1[0]=color;
	num=(xend-xsta)*(yend-ysta);
	LCD_Address_Set(xsta,ysta,xend-1,yend-1);//������ʾ��Χ
	delay_us(1);
	SPI_Cmd(SPI2, DISABLE);//ʹ��SPI
	SPI2->CR1|=1<<11;//����SPI16λ����ģʽ
	SPI_Cmd(SPI2, ENABLE);//ʹ��SPI
	delay_us(1);
	while(t){
		if(num>65534){
			num-=65534;
			num1=65534;
		}
		else{
			t=0;
			num1=num;
		}
		MYDMA_Config1(DMA1_Channel5,(u32)&SPI2->DR,(u32)color1,num1);
		SPI_I2S_DMACmd(SPI2,SPI_I2S_DMAReq_Tx,ENABLE);
		MYDMA_Enable(DMA1_Channel5);
		while(1){
			if(DMA_GetFlagStatus(DMA1_FLAG_TC5)!=RESET){//�ȴ�ͨ��4�������
				DMA_ClearFlag(DMA1_FLAG_TC5);//���ͨ��3������ɱ�־
				break;
			}
		}
  }
	delay_us(1);
	SPI2->CR1=~SPI2->CR1;
	SPI2->CR1|=1<<11;
	SPI2->CR1=~SPI2->CR1;//����SPI8λ����ģʽ
	SPI_Cmd(SPI2,ENABLE);//ʹ��SPI
	delay_us(1);
}

/*��ָ�����������ɫ
xsta,ysta   ��ʼ����
xend,yend   ��ֹ����
color       Ҫ������ɫ

***ˢ�����Ҫ1us��ʱʹ��spi8λ���ݴ���

//��GIF��ʾ��Ҫ
*/
void LCD_Color_Fill(u16 xsta,u16 ysta,u16 xend,u16 yend,u16 *color){
	u16 color1[1],t=1;
	u32 num,num1;
	color1[0]=*color;
	num=(xend-xsta)*(yend-ysta);
	LCD_Address_Set(xsta,ysta,xend-1,yend-1);//������ʾ��Χ
	delay_us(1);
	SPI_Cmd(SPI2, DISABLE);//ʹ��SPI
	SPI2->CR1|=1<<11;//����SPI16λ����ģʽ
	SPI_Cmd(SPI2, ENABLE);//ʹ��SPI
	delay_us(1);
	while(t){
		if(num>65534){
			num-=65534;
			num1=65534;
		}
		else{
			t=0;
			num1=num;
		}
		MYDMA_Config1(DMA1_Channel5,(u32)&SPI2->DR,(u32)color1,num1);
		SPI_I2S_DMACmd(SPI2,SPI_I2S_DMAReq_Tx,ENABLE);
		MYDMA_Enable(DMA1_Channel5);
		while(1){
			if(DMA_GetFlagStatus(DMA1_FLAG_TC5)!=RESET){//�ȴ�ͨ��4�������
				DMA_ClearFlag(DMA1_FLAG_TC5);//���ͨ��3������ɱ�־
				break;
			}
		}
  }
	delay_us(1);
	SPI2->CR1=~SPI2->CR1;
	SPI2->CR1|=1<<11;
	SPI2->CR1=~SPI2->CR1;//����SPI8λ����ģʽ
	SPI_Cmd(SPI2,ENABLE);//ʹ��SPI
	delay_us(1);
}

/*��ָ��λ�û���
x,y ��������
color �����ɫ
*/
void LCD_DrawPoint(u16 x,u16 y,u16 color){
	LCD_Address_Set(x,y,x,y);//���ù��λ��
	LCD_WR_DATA(color);
}


/*����
x1,y1   ��ʼ����
x2,y2   ��ֹ����
color   �ߵ���ɫ
*/
void LCD_DrawLine(u16 x1,u16 y1,u16 x2,u16 y2,u16 color){
	u16 t;
	int xerr=0,yerr=0,delta_x,delta_y,distance;
	int incx,incy,uRow,uCol;
	delta_x=x2-x1; //������������
	delta_y=y2-y1;
	uRow=x1;//�����������
	uCol=y1;
	if(delta_x>0)incx=1; //���õ�������
	else if (delta_x==0)incx=0;//��ֱ��
	else {incx=-1;delta_x=-delta_x;}
	if(delta_y>0)incy=1;
	else if (delta_y==0)incy=0;//ˮƽ��
	else {incy=-1;delta_y=-delta_y;}
	if(delta_x>delta_y)distance=delta_x; //ѡȡ��������������
	else distance=delta_y;
	for(t=0;t<distance+1;t++){
		LCD_DrawPoint(uRow,uCol,color);//����
		xerr+=delta_x;
		yerr+=delta_y;
		if(xerr>distance){
			xerr-=distance;
			uRow+=incx;
		}
		if(yerr>distance){
			yerr-=distance;
			uCol+=incy;
		}
	}
}


/*������
x1,y1   ��ʼ����
x2,y2   ��ֹ����
color   ���ε���ɫ
*/
void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2,u16 color){
	LCD_DrawLine(x1,y1,x2,y1,color);
	LCD_DrawLine(x1,y1,x1,y2,color);
	LCD_DrawLine(x1,y2,x2,y2,color);
	LCD_DrawLine(x2,y1,x2,y2,color);
}


/*��Բ
x0,y0   Բ������
r       �뾶
color   Բ����ɫ
*/
void Draw_Circle(u16 x0,u16 y0,u8 r,u16 color){
	int a,b;
	a=0;b=r;
	while(a<=b){
		LCD_DrawPoint(x0-b,y0-a,color);             //3
		LCD_DrawPoint(x0+b,y0-a,color);             //0
		LCD_DrawPoint(x0-a,y0+b,color);             //1
		LCD_DrawPoint(x0-a,y0-b,color);             //2
		LCD_DrawPoint(x0+b,y0+a,color);             //4
		LCD_DrawPoint(x0+a,y0-b,color);             //5
		LCD_DrawPoint(x0+a,y0+b,color);             //6
		LCD_DrawPoint(x0-b,y0+a,color);             //7
		a++;
		if((a*a+b*b)>(r*r)){//�ж�Ҫ���ĵ��Ƿ��Զ
			b--;
		}
	}
}

/*��ʾ���ִ�
x,y��ʾ����
*s Ҫ��ʾ�ĺ��ִ�
fc �ֵ���ɫ
bc �ֵı���ɫ
sizey �ֺ� ��ѡ 16 24 32
mode:  0�ǵ���ģʽ  1����ģʽ
*/
void LCD_ShowChinese(u16 x,u16 y,char *s,u16 fc,u16 bc,u8 sizey,u8 mode){
	while(*s!=0){
		if(sizey==12) LCD_ShowChinese12x12(x,y,s,fc,bc,sizey,mode);
		else if(sizey==16) LCD_ShowChinese16x16(x,y,s,fc,bc,sizey,mode);
		else if(sizey==24) LCD_ShowChinese24x24(x,y,s,fc,bc,sizey,mode);
		else if(sizey==32) LCD_ShowChinese32x32(x,y,s,fc,bc,sizey,mode);
		else return;
		s+=2;
		x+=sizey;
	}
}

/*��ʾ����12x12����
x,y��ʾ����
*s Ҫ��ʾ�ĺ���
fc �ֵ���ɫ
bc �ֵı���ɫ
sizey �ֺ�
mode:  0�ǵ���ģʽ  1����ģʽ
*/
void LCD_ShowChinese12x12(u16 x,u16 y,char *s,u16 fc,\
													u16 bc,u8 sizey,u8 mode){
	u8 i,j,m=0;
	u16 k;
	u16 HZnum;//������Ŀ
	u16 TypefaceNum;//һ���ַ���ռ�ֽڴ�С
	u16 x0=x;
	TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;

	HZnum=sizeof(tfont12)/sizeof(typFNT_GB12);	//ͳ�ƺ�����Ŀ
	for(k=0;k<HZnum;k++){
		if((tfont12[k].Index[0]==*(s))&&(tfont12[k].Index[1]==*(s+1))){
			LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
			for(i=0;i<TypefaceNum;i++){
				for(j=0;j<8;j++){
					if(!mode){//�ǵ��ӷ�ʽ
						if(tfont12[k].Msk[i]&(0x01<<j))LCD_WR_DATA(fc);
						else LCD_WR_DATA(bc);
						m++;
						if(m%sizey==0){
							m=0;
							break;
						}
					}
					else{//���ӷ�ʽ
						if(tfont12[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//��һ����
						x++;
						if((x-x0)==sizey){
							x=x0;
							y++;
							break;
						}
					}
				}
			}
		}
		continue;  //���ҵ���Ӧ�����ֿ������˳�����ֹ��������ظ�ȡģ����Ӱ��
	}
}

/*��ʾ����16x16����
x,y��ʾ����
*s Ҫ��ʾ�ĺ���
fc �ֵ���ɫ
bc �ֵı���ɫ
sizey �ֺ�
mode:  0�ǵ���ģʽ  1����ģʽ
*/
void LCD_ShowChinese16x16(u16 x,u16 y,char *s,u16 fc,\
													u16 bc,u8 sizey,u8 mode){
	u8 i,j,m=0;
	u16 k;
	u16 HZnum;		//������Ŀ
	u16 TypefaceNum;		//һ���ַ���ռ�ֽڴ�С
	u16 x0=x;
  	TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
	HZnum=sizeof(tfont16)/sizeof(typFNT_GB16);		//ͳ�ƺ�����Ŀ
	for(k=0;k<HZnum;k++){
		if ((tfont16[k].Index[0]==*(s))&&(tfont16[k].Index[1]==*(s+1))){
			LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
			for(i=0;i<TypefaceNum;i++){
				for(j=0;j<8;j++){
					if(!mode){		//�ǵ��ӷ�ʽ
						if(tfont16[k].Msk[i]&(0x01<<j))LCD_WR_DATA(fc);
						else LCD_WR_DATA(bc);
						m++;
						if(m%sizey==0){
							m=0;
							break;
						}
					}
					else{		//���ӷ�ʽ
						if(tfont16[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);		//��һ����
						x++;
						if((x-x0)==sizey){
							x=x0;
							y++;
							break;
						}
					}
				}
			}
		}
		continue;		//���ҵ���Ӧ�����ֿ������˳�����ֹ��������ظ�ȡģ����Ӱ��
	}
}


/*��ʾ����24x24����
x,y��ʾ����
*s Ҫ��ʾ�ĺ���
fc �ֵ���ɫ
bc �ֵı���ɫ
sizey �ֺ�
mode:  0�ǵ���ģʽ  1����ģʽ
*/
void LCD_ShowChinese24x24(u16 x,u16 y,char *s,u16 fc,u16 bc,u8 sizey,u8 mode){
	u8 i,j,m=0;
	u16 k;
	u16 HZnum;//������Ŀ
	u16 TypefaceNum;//һ���ַ���ռ�ֽڴ�С
	u16 x0=x;
	TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
	HZnum=sizeof(tfont24)/sizeof(typFNT_GB24);	//ͳ�ƺ�����Ŀ
	for(k=0;k<HZnum;k++){
		if ((tfont24[k].Index[0]==*(s))&&(tfont24[k].Index[1]==*(s+1))){
			LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
			for(i=0;i<TypefaceNum;i++){
				for(j=0;j<8;j++){
					if(!mode){//�ǵ��ӷ�ʽ
						if(tfont24[k].Msk[i]&(0x01<<j))LCD_WR_DATA(fc);
						else LCD_WR_DATA(bc);
						m++;
						if(m%sizey==0){
							m=0;
							break;
						}
					}
					else{//���ӷ�ʽ
						if(tfont24[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//��һ����
						x++;
						if((x-x0)==sizey){
							x=x0;
							y++;
							break;
						}
					}
				}
			}
		}
		continue;  //���ҵ���Ӧ�����ֿ������˳�����ֹ��������ظ�ȡģ����Ӱ��
	}
}

/*��ʾ����32x32����
x,y��ʾ����
*s Ҫ��ʾ�ĺ���
fc �ֵ���ɫ
bc �ֵı���ɫ
sizey �ֺ�
mode:  0�ǵ���ģʽ  1����ģʽ
*/
void LCD_ShowChinese32x32(u16 x,u16 y,char *s,u16 fc,\
													u16 bc,u8 sizey,u8 mode){
	u8 i,j,m=0;
	u16 k;
	u16 HZnum;//������Ŀ
	u16 TypefaceNum;//һ���ַ���ռ�ֽڴ�С
	u16 x0=x;
	TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
	HZnum=sizeof(tfont32)/sizeof(typFNT_GB32);	//ͳ�ƺ�����Ŀ
	for(k=0;k<HZnum;k++){
		if ((tfont32[k].Index[0]==*(s))&&(tfont32[k].Index[1]==*(s+1))){
			LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
			for(i=0;i<TypefaceNum;i++){
				for(j=0;j<8;j++){
					if(!mode){//�ǵ��ӷ�ʽ
						if(tfont32[k].Msk[i]&(0x01<<j))LCD_WR_DATA(fc);
						else LCD_WR_DATA(bc);
						m++;
						if(m%sizey==0){
							m=0;
							break;
						}
					}
					else{//���ӷ�ʽ
						if(tfont32[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//��һ����
						x++;
						if((x-x0)==sizey){
							x=x0;
							y++;
							break;
						}
					}
				}
			}
		}
		continue;  //���ҵ���Ӧ�����ֿ������˳�����ֹ��������ظ�ȡģ����Ӱ��
	}
}


/*��ʾ�����ַ�
x,y��ʾ����
num Ҫ��ʾ���ַ�
fc �ֵ���ɫ
bc �ֵı���ɫ
sizey �ֺ�
mode:  0�ǵ���ģʽ  1����ģʽ
*/
void LCD_ShowChar(u16 x,u16 y,char num,u16 fc,u16 bc,u8 sizey,u8 mode){
	u8 temp,sizex,t,m=0;
	u16 i,TypefaceNum;//һ���ַ���ռ�ֽڴ�С
	u16 x0=x;
	sizex=sizey/2;
	TypefaceNum=(sizex/8+((sizex%8)?1:0))*sizey;
	num=num-' ';    //�õ�ƫ�ƺ��ֵ
	LCD_Address_Set(x,y,x+sizex-1,y+sizey-1);  //���ù��λ��
	for(i=0;i<TypefaceNum;i++){
		if(sizey==12)temp=ascii_1206[num][i];		       //����6x12����
		else if(sizey==16)temp=ascii_1608[num][i];		 //����8x16����
		else if(sizey==24)temp=ascii_2412[num][i];		 //����12x24����
		else if(sizey==32)temp=ascii_3216[num][i];		 //����16x32����
		else return;
		for(t=0;t<8;t++){
			if(!mode){//�ǵ���ģʽ
				if(temp&(0x01<<t))LCD_WR_DATA(fc);
				else LCD_WR_DATA(bc);
				m++;
				if(m%sizex==0){
					m=0;
					break;
				}
			}
			else{//����ģʽ
				if(temp&(0x01<<t))LCD_DrawPoint(x,y,fc);//��һ����
				x++;
				if((x-x0)==sizex){
					x=x0;
					y++;
					break;
				}
			}
		}
	}
}


/*��ʾ�ַ���
x,y��ʾ����
*p Ҫ��ʾ���ַ���
fc �ֵ���ɫ
bc �ֵı���ɫ
sizey �ֺ�
mode:  0�ǵ���ģʽ  1����ģʽ
*/
void LCD_ShowString(u16 x,u16 y,const char *p,u16 fc,\
										u16 bc,u8 sizey,u8 mode){
	unsigned char p_num=0;
	while(*p!='\0'){
		if(p_num<28)
		{
			LCD_ShowChar(x,y,*p,fc,bc,sizey,mode);
			x+=16/2;
		}
		else if(p_num>=28){
			p_num=0;
			x=0;
			y+=16;
			LCD_ShowChar(x,y,*p,fc,bc,sizey,mode);
			x+=16/2;
		}
		p++;
		p_num++;
	}
}
/*��ʾ����
m������nָ��
*/
u32 mypow(u8 m,u8 n){
	u32 result=1;
	while(n--)result*=m;
	return result;
}


/*��ʾ��������
x,y��ʾ����
num Ҫ��ʾ��������
len Ҫ��ʾ��λ��
fc �ֵ���ɫ
bc �ֵı���ɫ
sizey �ֺ�
*/
void LCD_ShowIntNum(u16 x,u16 y,char num,u8 len,u16 fc,u16 bc,u8 sizey){
	u8 t,temp;
	u8 enshow=0;
	u8 sizex=sizey/2;
	for(t=0;t<len;t++){
		temp=(num/mypow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1)){
			if(temp==0){
				LCD_ShowChar(x+t*sizex,y,' ',fc,bc,sizey,0);
				continue;
			}else enshow=1;

		}
	 	LCD_ShowChar(x+t*sizex,y,temp+48,fc,bc,sizey,0);
	}
}


/*��ʾ��λС������
x,y��ʾ����
num Ҫ��ʾС������
len Ҫ��ʾ��λ��
fc �ֵ���ɫ
bc �ֵı���ɫ
sizey �ֺ�
*/
void LCD_ShowFloatNum1(u16 x,u16 y,float num,\
											 u8 len,u16 fc,u16 bc,u8 sizey){
	u8 t,temp,sizex;
	u16 num1;
	sizex=sizey/2;
	num1=num*100;
	for(t=0;t<len;t++){
		temp=(num1/mypow(10,len-t-1))%10;
		if(t==(len-2)){
			LCD_ShowChar(x+(len-2)*sizex,y,'.',fc,bc,sizey,0);
			t++;
			len+=1;
		}
	 	LCD_ShowChar(x+t*sizex,y,temp+48,fc,bc,sizey,0);
	}
}


/*��ʾͼƬ
x,y�������
length ͼƬ����
width  ͼƬ���
pic[]  ͼƬ����
*/
void LCD_ShowPicture(u16 x,u16 y,u16 length,u16 width,const u8 pic[]){
	u8 t=1;
	u32 num=length*width*2,num1;
	LCD_Address_Set(x,y,x+length-1,y+width-1);
	while(t){
	  if(num>65534){
			num-=65534;
			num1=65534;
		}
		else{
			t=0;
			num1=num;
		}
		MYDMA_Config(DMA1_Channel5,(u32)&SPI2->DR,(u32)pic,num1);
		SPI_I2S_DMACmd(SPI2,SPI_I2S_DMAReq_Tx,ENABLE);
		MYDMA_Enable(DMA1_Channel5);
		while(1){
			if(DMA_GetFlagStatus(DMA1_FLAG_TC5)!=RESET){
				DMA_ClearFlag(DMA1_FLAG_TC5);
				break;
			}
		}
		pic+=65534;
	}
}

/*��ʾ�ַ���
x,y��ʾ����
*p Ҫ��ʾ���ַ���
*/
void LCDDebug(u16 x,u16 y,char *p){
	unsigned char p_num=0;
	while(*p!='\0'){
		
		if(p_num<28)
		{
			LCD_ShowChar(x,y,*p,WHITE,BLACK,16,0);
			x+=16/2;
		}
		else if(p_num>=28){
			p_num=0;
			x=0;
			y+=16;
			LCD_ShowChar(x,y,*p,WHITE,BLACK,16,0);
			x+=16/2;
		}
		p++;
		p_num++;
	}
}
