#include "TFT.h"
#include "main.h"
#include "TFT_lcdfont.h"

DMA_InitTypeDef DMA_InitStructure;

u16 DMA1_MEM_LEN;//保存DMA每次数据传送的长度

/*DMA1的各通道配置
这里的传输形式是固定的,这点要根据不同的情况来修改
从存储器->外设模式/8位数据宽度/存储器增量模式
DMA_CHx:DMA通道CHx
cpar:外设地址
cmar:存储器地址
cndtr:数据传输量 */
static void MYDMA_Config(DMA_Channel_TypeDef* DMA_CHx,u32 cpar,u32 cmar,u16 cndtr){
 	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	//使能DMA传输

	DMA_DeInit(DMA_CHx);   //将DMA的通道1寄存器重设为缺省值
	DMA1_MEM_LEN=cndtr;
	DMA_InitStructure.DMA_PeripheralBaseAddr = cpar;  //DMA外设ADC基地址
	DMA_InitStructure.DMA_MemoryBaseAddr = cmar;  //DMA内存基地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;  //数据传输方向，从内存读取发送到外设
	DMA_InitStructure.DMA_BufferSize = cndtr;  //DMA通道的DMA缓存的大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //外设地址寄存器不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //内存地址寄存器递增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //数据宽度为8位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //数据宽度为8位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  //工作在正常缓存模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; //DMA通道 x拥有中优先级
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //DMA通道x没有设置为内存到内存传输
	DMA_Init(DMA_CHx, &DMA_InitStructure);  //根据DMA_InitStruct中指定的参数初始化DMA的通道USART1_Tx_DMA_Channel所标识的寄存器
}

static void MYDMA_Config1(DMA_Channel_TypeDef* DMA_CHx,u32 cpar,u32 cmar,u16 cndtr){
 	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);	//使能DMA传输

  DMA_DeInit(DMA_CHx);   //将DMA的通道1寄存器重设为缺省值
	DMA1_MEM_LEN=cndtr;
	DMA_InitStructure.DMA_PeripheralBaseAddr = cpar;  //DMA外设ADC基地址
	DMA_InitStructure.DMA_MemoryBaseAddr = cmar;  //DMA内存基地址
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;  //数据传输方向，从内存读取发送到外设
	DMA_InitStructure.DMA_BufferSize = cndtr;  //DMA通道的DMA缓存的大小
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //外设地址寄存器不变
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;  //内存地址寄存器不变
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;  //数据宽度为16位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord; //数据宽度为16位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  //工作在正常缓存模式
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; //DMA通道 x拥有中优先级
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;  //DMA通道x没有设置为内存到内存传输
	DMA_Init(DMA_CHx, &DMA_InitStructure);  //根据DMA_InitStruct中指定的参数初始化DMA的通道USART1_Tx_DMA_Channel所标识的寄存器
}

//开启一次DMA传输
static void MYDMA_Enable(DMA_Channel_TypeDef*DMA_CHx){
	DMA_Cmd(DMA_CHx, DISABLE );
 	DMA_SetCurrDataCounter(DMA1_Channel5,DMA1_MEM_LEN);
 	DMA_Cmd(DMA_CHx, ENABLE);
}

static void TFT_SPI_Init(void){
	GPIO_InitTypeDef TFT_SPI_IO_Init;
	SPI_InitTypeDef TFT_SPI_InitTypeDef;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);		//PORTB时钟使能
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD,ENABLE);		//PORTB时钟使能
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,ENABLE);		//SPI2时钟使能

	TFT_SPI_IO_Init.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_15;
	TFT_SPI_IO_Init.GPIO_Mode = GPIO_Mode_AF_PP;  //PB13/15复用推挽输出
	TFT_SPI_IO_Init.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &TFT_SPI_IO_Init);//初始化GPIOB
	GPIO_SetBits(GPIOB,GPIO_Pin_13|GPIO_Pin_15);  //PB13/15上拉

	TFT_SPI_InitTypeDef.SPI_Direction = SPI_Direction_1Line_Tx;//只发送
	TFT_SPI_InitTypeDef.SPI_Mode = SPI_Mode_Master;//主
	TFT_SPI_InitTypeDef.SPI_DataSize = SPI_DataSize_8b;//8位帧
	TFT_SPI_InitTypeDef.SPI_CPOL = SPI_CPOL_High;//空闲时SCLK高
	TFT_SPI_InitTypeDef.SPI_CPHA = SPI_CPHA_2Edge;//串行同步时钟空第二个时钟沿捕获
	TFT_SPI_InitTypeDef.SPI_NSS = SPI_NSS_Soft;//NSS信号软件管理
	TFT_SPI_InitTypeDef.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;//预分频值为2
	TFT_SPI_InitTypeDef.SPI_FirstBit = SPI_FirstBit_MSB;//数据高位先行
	TFT_SPI_InitTypeDef.SPI_CRCPolynomial = 7;//CRC值计算的多项式
	SPI_Init(SPI2,&TFT_SPI_InitTypeDef);
	SPI_Cmd(SPI2, ENABLE);

	delay_ms(1);

	TFT_SPI_IO_Init.GPIO_Pin = TFT_BLK | TFT_DC | TFT_RES;
	TFT_SPI_IO_Init.GPIO_Mode = GPIO_Mode_Out_PP;
	TFT_SPI_IO_Init.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &TFT_SPI_IO_Init);
	GPIO_SetBits(GPIOD,TFT_BLK | TFT_DC | TFT_RES);
}


/*SPI发数据*/
static void LCD_Writ_Bus(u8 dat){
  while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE) == RESET);//检查接收标志位
	SPI_I2S_SendData(SPI2,dat);
	delay_us(1);
}
/*TFT写8位*/
static void LCD_WR_DATA8(u8 dat){
	LCD_Writ_Bus(dat);
}
/*TFT写16位数据*/
static void LCD_WR_DATA(u16 dat){
	LCD_Writ_Bus(dat>>8);
	LCD_Writ_Bus(dat);
}
/*TFT写命令*/
static void LCD_WR_REG(u8 dat){
	LCD_DC_Clr();//写命令
	LCD_Writ_Bus(dat);
	LCD_DC_Set();//写数据
}
/*起始和结束地址
x1,x2 设置列的起始和结束地址
y1,y2 设置行的起始和结束地址
*/
void LCD_Address_Set(u16 x1,u16 y1,u16 x2,u16 y2){
	if(USE_HORIZONTAL==0){
		LCD_WR_REG(0x2a);//列地址设置
		LCD_WR_DATA(x1);
		LCD_WR_DATA(x2);
		LCD_WR_REG(0x2b);//行地址设置
		LCD_WR_DATA(y1);
		LCD_WR_DATA(y2);
		LCD_WR_REG(0x2c);//储存器写
	}
	else if(USE_HORIZONTAL==1){
		LCD_WR_REG(0x2a);//列地址设置
		LCD_WR_DATA(x1);
		LCD_WR_DATA(x2);
		LCD_WR_REG(0x2b);//行地址设置
		LCD_WR_DATA(y1+80);
		LCD_WR_DATA(y2+80);
		LCD_WR_REG(0x2c);//储存器写
	}
	else if(USE_HORIZONTAL==2){
		LCD_WR_REG(0x2a);//列地址设置
		LCD_WR_DATA(x1);
		LCD_WR_DATA(x2);
		LCD_WR_REG(0x2b);//行地址设置
		LCD_WR_DATA(y1);
		LCD_WR_DATA(y2);
		LCD_WR_REG(0x2c);//储存器写
	}
	else{
		LCD_WR_REG(0x2a);//列地址设置
		LCD_WR_DATA(x1+80);
		LCD_WR_DATA(x2+80);
		LCD_WR_REG(0x2b);//行地址设置
		LCD_WR_DATA(y1);
		LCD_WR_DATA(y2);
		LCD_WR_REG(0x2c);//储存器写
	}
}

void LCD_Init(void){
	TFT_SPI_Init();

	LCD_RES_Clr();//复位
	delay_ms(100);
	LCD_RES_Set();
	delay_ms(100);

	LCD_BLK_Set();//打开背光
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

/*在指定区域填充颜色
xsta,ysta   起始坐标
xend,yend   终止坐标
color       要填充的颜色

***刷屏后必要1us延时使能spi8位数据传输
*/
void LCD_Fill(u16 xsta,u16 ysta,u16 xend,u16 yend,u16 color){
	u16 color1[1],t=1;
	u32 num,num1;
	color1[0]=color;
	num=(xend-xsta)*(yend-ysta);
	LCD_Address_Set(xsta,ysta,xend-1,yend-1);//设置显示范围
	delay_us(1);
	SPI_Cmd(SPI2, DISABLE);//使能SPI
	SPI2->CR1|=1<<11;//设置SPI16位传输模式
	SPI_Cmd(SPI2, ENABLE);//使能SPI
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
			if(DMA_GetFlagStatus(DMA1_FLAG_TC5)!=RESET){//等待通道4传输完成
				DMA_ClearFlag(DMA1_FLAG_TC5);//清除通道3传输完成标志
				break;
			}
		}
  }
	delay_us(1);
	SPI2->CR1=~SPI2->CR1;
	SPI2->CR1|=1<<11;
	SPI2->CR1=~SPI2->CR1;//设置SPI8位传输模式
	SPI_Cmd(SPI2,ENABLE);//使能SPI
	delay_us(1);
}

/*在指定区域填充颜色
xsta,ysta   起始坐标
xend,yend   终止坐标
color       要填充的颜色

***刷屏后必要1us延时使能spi8位数据传输

//仅GIF显示需要
*/
void LCD_Color_Fill(u16 xsta,u16 ysta,u16 xend,u16 yend,u16 *color){
	u16 color1[1],t=1;
	u32 num,num1;
	color1[0]=*color;
	num=(xend-xsta)*(yend-ysta);
	LCD_Address_Set(xsta,ysta,xend-1,yend-1);//设置显示范围
	delay_us(1);
	SPI_Cmd(SPI2, DISABLE);//使能SPI
	SPI2->CR1|=1<<11;//设置SPI16位传输模式
	SPI_Cmd(SPI2, ENABLE);//使能SPI
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
			if(DMA_GetFlagStatus(DMA1_FLAG_TC5)!=RESET){//等待通道4传输完成
				DMA_ClearFlag(DMA1_FLAG_TC5);//清除通道3传输完成标志
				break;
			}
		}
  }
	delay_us(1);
	SPI2->CR1=~SPI2->CR1;
	SPI2->CR1|=1<<11;
	SPI2->CR1=~SPI2->CR1;//设置SPI8位传输模式
	SPI_Cmd(SPI2,ENABLE);//使能SPI
	delay_us(1);
}

/*在指定位置画点
x,y 画点坐标
color 点的颜色
*/
void LCD_DrawPoint(u16 x,u16 y,u16 color){
	LCD_Address_Set(x,y,x,y);//设置光标位置
	LCD_WR_DATA(color);
}


/*画线
x1,y1   起始坐标
x2,y2   终止坐标
color   线的颜色
*/
void LCD_DrawLine(u16 x1,u16 y1,u16 x2,u16 y2,u16 color){
	u16 t;
	int xerr=0,yerr=0,delta_x,delta_y,distance;
	int incx,incy,uRow,uCol;
	delta_x=x2-x1; //计算坐标增量
	delta_y=y2-y1;
	uRow=x1;//画线起点坐标
	uCol=y1;
	if(delta_x>0)incx=1; //设置单步方向
	else if (delta_x==0)incx=0;//垂直线
	else {incx=-1;delta_x=-delta_x;}
	if(delta_y>0)incy=1;
	else if (delta_y==0)incy=0;//水平线
	else {incy=-1;delta_y=-delta_y;}
	if(delta_x>delta_y)distance=delta_x; //选取基本增量坐标轴
	else distance=delta_y;
	for(t=0;t<distance+1;t++){
		LCD_DrawPoint(uRow,uCol,color);//画点
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


/*画矩形
x1,y1   起始坐标
x2,y2   终止坐标
color   矩形的颜色
*/
void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2,u16 color){
	LCD_DrawLine(x1,y1,x2,y1,color);
	LCD_DrawLine(x1,y1,x1,y2,color);
	LCD_DrawLine(x1,y2,x2,y2,color);
	LCD_DrawLine(x2,y1,x2,y2,color);
}


/*画圆
x0,y0   圆心坐标
r       半径
color   圆的颜色
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
		if((a*a+b*b)>(r*r)){//判断要画的点是否过远
			b--;
		}
	}
}

/*显示汉字串
x,y显示坐标
*s 要显示的汉字串
fc 字的颜色
bc 字的背景色
sizey 字号 可选 16 24 32
mode:  0非叠加模式  1叠加模式
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

/*显示单个12x12汉字
x,y显示坐标
*s 要显示的汉字
fc 字的颜色
bc 字的背景色
sizey 字号
mode:  0非叠加模式  1叠加模式
*/
void LCD_ShowChinese12x12(u16 x,u16 y,char *s,u16 fc,\
													u16 bc,u8 sizey,u8 mode){
	u8 i,j,m=0;
	u16 k;
	u16 HZnum;//汉字数目
	u16 TypefaceNum;//一个字符所占字节大小
	u16 x0=x;
	TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;

	HZnum=sizeof(tfont12)/sizeof(typFNT_GB12);	//统计汉字数目
	for(k=0;k<HZnum;k++){
		if((tfont12[k].Index[0]==*(s))&&(tfont12[k].Index[1]==*(s+1))){
			LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
			for(i=0;i<TypefaceNum;i++){
				for(j=0;j<8;j++){
					if(!mode){//非叠加方式
						if(tfont12[k].Msk[i]&(0x01<<j))LCD_WR_DATA(fc);
						else LCD_WR_DATA(bc);
						m++;
						if(m%sizey==0){
							m=0;
							break;
						}
					}
					else{//叠加方式
						if(tfont12[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//画一个点
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
		continue;  //查找到对应点阵字库立即退出，防止多个汉字重复取模带来影响
	}
}

/*显示单个16x16汉字
x,y显示坐标
*s 要显示的汉字
fc 字的颜色
bc 字的背景色
sizey 字号
mode:  0非叠加模式  1叠加模式
*/
void LCD_ShowChinese16x16(u16 x,u16 y,char *s,u16 fc,\
													u16 bc,u8 sizey,u8 mode){
	u8 i,j,m=0;
	u16 k;
	u16 HZnum;		//汉字数目
	u16 TypefaceNum;		//一个字符所占字节大小
	u16 x0=x;
  	TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
	HZnum=sizeof(tfont16)/sizeof(typFNT_GB16);		//统计汉字数目
	for(k=0;k<HZnum;k++){
		if ((tfont16[k].Index[0]==*(s))&&(tfont16[k].Index[1]==*(s+1))){
			LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
			for(i=0;i<TypefaceNum;i++){
				for(j=0;j<8;j++){
					if(!mode){		//非叠加方式
						if(tfont16[k].Msk[i]&(0x01<<j))LCD_WR_DATA(fc);
						else LCD_WR_DATA(bc);
						m++;
						if(m%sizey==0){
							m=0;
							break;
						}
					}
					else{		//叠加方式
						if(tfont16[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);		//画一个点
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
		continue;		//查找到对应点阵字库立即退出，防止多个汉字重复取模带来影响
	}
}


/*显示单个24x24汉字
x,y显示坐标
*s 要显示的汉字
fc 字的颜色
bc 字的背景色
sizey 字号
mode:  0非叠加模式  1叠加模式
*/
void LCD_ShowChinese24x24(u16 x,u16 y,char *s,u16 fc,u16 bc,u8 sizey,u8 mode){
	u8 i,j,m=0;
	u16 k;
	u16 HZnum;//汉字数目
	u16 TypefaceNum;//一个字符所占字节大小
	u16 x0=x;
	TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
	HZnum=sizeof(tfont24)/sizeof(typFNT_GB24);	//统计汉字数目
	for(k=0;k<HZnum;k++){
		if ((tfont24[k].Index[0]==*(s))&&(tfont24[k].Index[1]==*(s+1))){
			LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
			for(i=0;i<TypefaceNum;i++){
				for(j=0;j<8;j++){
					if(!mode){//非叠加方式
						if(tfont24[k].Msk[i]&(0x01<<j))LCD_WR_DATA(fc);
						else LCD_WR_DATA(bc);
						m++;
						if(m%sizey==0){
							m=0;
							break;
						}
					}
					else{//叠加方式
						if(tfont24[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//画一个点
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
		continue;  //查找到对应点阵字库立即退出，防止多个汉字重复取模带来影响
	}
}

/*显示单个32x32汉字
x,y显示坐标
*s 要显示的汉字
fc 字的颜色
bc 字的背景色
sizey 字号
mode:  0非叠加模式  1叠加模式
*/
void LCD_ShowChinese32x32(u16 x,u16 y,char *s,u16 fc,\
													u16 bc,u8 sizey,u8 mode){
	u8 i,j,m=0;
	u16 k;
	u16 HZnum;//汉字数目
	u16 TypefaceNum;//一个字符所占字节大小
	u16 x0=x;
	TypefaceNum=(sizey/8+((sizey%8)?1:0))*sizey;
	HZnum=sizeof(tfont32)/sizeof(typFNT_GB32);	//统计汉字数目
	for(k=0;k<HZnum;k++){
		if ((tfont32[k].Index[0]==*(s))&&(tfont32[k].Index[1]==*(s+1))){
			LCD_Address_Set(x,y,x+sizey-1,y+sizey-1);
			for(i=0;i<TypefaceNum;i++){
				for(j=0;j<8;j++){
					if(!mode){//非叠加方式
						if(tfont32[k].Msk[i]&(0x01<<j))LCD_WR_DATA(fc);
						else LCD_WR_DATA(bc);
						m++;
						if(m%sizey==0){
							m=0;
							break;
						}
					}
					else{//叠加方式
						if(tfont32[k].Msk[i]&(0x01<<j))	LCD_DrawPoint(x,y,fc);//画一个点
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
		continue;  //查找到对应点阵字库立即退出，防止多个汉字重复取模带来影响
	}
}


/*显示单个字符
x,y显示坐标
num 要显示的字符
fc 字的颜色
bc 字的背景色
sizey 字号
mode:  0非叠加模式  1叠加模式
*/
void LCD_ShowChar(u16 x,u16 y,char num,u16 fc,u16 bc,u8 sizey,u8 mode){
	u8 temp,sizex,t,m=0;
	u16 i,TypefaceNum;//一个字符所占字节大小
	u16 x0=x;
	sizex=sizey/2;
	TypefaceNum=(sizex/8+((sizex%8)?1:0))*sizey;
	num=num-' ';    //得到偏移后的值
	LCD_Address_Set(x,y,x+sizex-1,y+sizey-1);  //设置光标位置
	for(i=0;i<TypefaceNum;i++){
		if(sizey==12)temp=ascii_1206[num][i];		       //调用6x12字体
		else if(sizey==16)temp=ascii_1608[num][i];		 //调用8x16字体
		else if(sizey==24)temp=ascii_2412[num][i];		 //调用12x24字体
		else if(sizey==32)temp=ascii_3216[num][i];		 //调用16x32字体
		else return;
		for(t=0;t<8;t++){
			if(!mode){//非叠加模式
				if(temp&(0x01<<t))LCD_WR_DATA(fc);
				else LCD_WR_DATA(bc);
				m++;
				if(m%sizex==0){
					m=0;
					break;
				}
			}
			else{//叠加模式
				if(temp&(0x01<<t))LCD_DrawPoint(x,y,fc);//画一个点
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


/*显示字符串
x,y显示坐标
*p 要显示的字符串
fc 字的颜色
bc 字的背景色
sizey 字号
mode:  0非叠加模式  1叠加模式
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
/*显示数字
m底数，n指数
*/
u32 mypow(u8 m,u8 n){
	u32 result=1;
	while(n--)result*=m;
	return result;
}


/*显示整数变量
x,y显示坐标
num 要显示整数变量
len 要显示的位数
fc 字的颜色
bc 字的背景色
sizey 字号
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


/*显示两位小数变量
x,y显示坐标
num 要显示小数变量
len 要显示的位数
fc 字的颜色
bc 字的背景色
sizey 字号
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


/*显示图片
x,y起点坐标
length 图片长度
width  图片宽度
pic[]  图片数组
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

/*显示字符串
x,y显示坐标
*p 要显示的字符串
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
