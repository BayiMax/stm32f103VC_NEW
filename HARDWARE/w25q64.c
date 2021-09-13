#include "w25q64.h"
#include "main.h"


u16 W25QXX_TYPE=GD25Q64;

void W25Qxx_SPIAndIO_Init(void){
	GPIO_InitTypeDef W25Qxx_IO;
	SPI_InitTypeDef W25Qxx_SPI;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD,ENABLE);

	W25Qxx_IO.GPIO_Pin = GPIO_Pin_7;
	W25Qxx_IO.GPIO_Mode = GPIO_Mode_Out_PP;
	W25Qxx_IO.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD,&W25Qxx_IO);
	GPIO_SetBits(GPIOD,GPIO_Pin_7);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);//要先开时钟，再重映射；这句表示关闭jtag，使能swd.
	
	W25Qxx_IO.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_5;
	W25Qxx_IO.GPIO_Mode = GPIO_Mode_AF_PP;
	W25Qxx_IO.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&W25Qxx_IO);
	GPIO_SetBits(GPIOB,GPIO_Pin_3|GPIO_Pin_5);
	
	W25Qxx_IO.GPIO_Pin = GPIO_Pin_4;
	W25Qxx_IO.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOB,&W25Qxx_IO);
	
	do{uint32_t tmpreg = AFIO->MAPR;
		 tmpreg |= AFIO_MAPR_SWJ_CFG;
		 tmpreg |= AFIO_MAPR_SPI1_REMAP;
		 AFIO->MAPR = tmpreg;
	}while(0u);
	
	W25Qxx_SPI.SPI_Direction = SPI_Direction_2Lines_FullDuplex;		//spi双向全双工
	W25Qxx_SPI.SPI_Mode = SPI_Mode_Master;		//主
	W25Qxx_SPI.SPI_DataSize = SPI_DataSize_8b;		//数据8位
	W25Qxx_SPI.SPI_CPOL = SPI_CPOL_High;		//时钟空闲为高
	W25Qxx_SPI.SPI_CPHA = SPI_CPHA_2Edge;		//时钟第二个跳变数据采样
	W25Qxx_SPI.SPI_NSS = SPI_NSS_Soft;		//nss信号由ssi控制
	W25Qxx_SPI.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;		//分频256
	W25Qxx_SPI.SPI_FirstBit = SPI_FirstBit_MSB;		//数据从MSB开始
	W25Qxx_SPI.SPI_CRCPolynomial = 7;		//CRC多项式
	W25QXX_CS=1;
	SPI_Init(SPI1,&W25Qxx_SPI);		//初始化SPI1
	SPI_Cmd(SPI1,ENABLE);		//使能SPI1
	SPI1_ReadWriteByte(0xff);		//启动传输
	SPI1_SetSpeed(SPI_BaudRatePrescaler_2);
//	W25QXX_TYPE=W25QXX_ReadID();
}
/*spi速度设定*/
void SPI1_SetSpeed(unsigned char SPI_BaudRatePrescaler){
	assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));
	SPI1->CR1&=0xffc7;
	SPI1->CR1|=SPI_BaudRatePrescaler;
	SPI_Cmd(SPI1,ENABLE);
}

/*SPIx 读写一个字节
TxData:要写入的字节
返回值:读取到的字节*/
u8 SPI1_ReadWriteByte(u8 TxData){
	u8 retry=0;
	while (SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_TXE) == RESET){		//检查指定的SPI标志位设置与否:发送缓存空标志位
		retry++;
		if(retry>200)return 0;
	}
	SPI_I2S_SendData(SPI1, TxData);		//通过外设SPIx发送一个数据
	retry=0;

	while (SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE) == RESET){		//检查指定的SPI标志位设置与否:接受缓存非空标志位
		retry++;
		if(retry>200)return 0;
	}
	return SPI_I2S_ReceiveData(SPI1);		//返回通过SPIx最近接收的数据
}

/*读取WQxx的状态寄存器
BIT 7  6  5   4   3   2   1   0
	spr rv tb bp2 bp1 bp0 wel busy
spr:默认0
BUSY:忙标记位（1，忙；0.空闲）
默认0x00*/
unsigned char W25QXX_ReadSR(void){
	u8 byte=0;
	W25QXX_CS=0;		//使能器件
	SPI1_ReadWriteByte(W25X_ReadStatusReg);		//发送读取状态寄存器命令
	byte=SPI1_ReadWriteByte(0Xff);		//读取一个字节
	W25QXX_CS=1;		//取消片选
	return byte;
}
/*写W25QXX状态寄存器
只有SPR,TB,BP2,BP1,BP0(bit 7,5,4,3,2)可以写!!!*/
void W25QXX_Write_SR(u8 sr){
	W25QXX_CS=0;		//使能器件
	SPI1_ReadWriteByte(W25X_WriteStatusReg);		//发送写取状态寄存器命令
	SPI1_ReadWriteByte(sr);		//写入一个字节
	W25QXX_CS=1;		//取消片选
}
/*W25QXX写使能
将WEL置位*/
void W25QXX_Write_Enable(void){
	W25QXX_CS=0;		//使能器件
	SPI1_ReadWriteByte(W25X_WriteEnable);		//发送写使能
	W25QXX_CS=1;		//取消片选
}
/*W25QXX写禁止
将WEL清零*/
void W25QXX_Write_Disable(void){
	W25QXX_CS=0;		//使能器件
	SPI1_ReadWriteByte(W25X_WriteDisable);		//发送写禁止指令
	W25QXX_CS=1;		//取消片选
}
/*读取芯片ID
返回值如下:
0XEF13,表示芯片型号为W25Q80
0XEF14,表示芯片型号为W25Q16
0XEF15,表示芯片型号为W25Q32
0XEF16,表示芯片型号为W25Q64
0XEF17,表示芯片型号为W25Q128*/
u16 W25QXX_ReadID(void){
	u16 Temp = 0;
	W25QXX_CS=0;
	SPI1_ReadWriteByte(0x90);		//发送读取ID命令
	SPI1_ReadWriteByte(0x00);
	SPI1_ReadWriteByte(0x00);
	SPI1_ReadWriteByte(0x00);
	Temp|=SPI1_ReadWriteByte(0xFF)<<8;
	Temp|=SPI1_ReadWriteByte(0xFF);
	W25QXX_CS=1;
	return Temp;
}
/*读取SPI FLASH
在指定地址开始读取指定长度的数据
pBuffer:数据存储区
ReadAddr:开始读取的地址(24bit)
NumByteToRead:要读取的字节数(最大65535)*/
void W25QXX_Read(u8* pBuffer,u32 ReadAddr,u16 NumByteToRead){
 	u16 i;
	W25QXX_CS=0;		//使能器件
	SPI1_ReadWriteByte(W25X_ReadData);		//发送读取命令
	SPI1_ReadWriteByte((u8)((ReadAddr)>>16));		//发送24bit地址
	SPI1_ReadWriteByte((u8)((ReadAddr)>>8));
	SPI1_ReadWriteByte((u8)ReadAddr);
	for(i=0;i<NumByteToRead;i++){
		pBuffer[i]=SPI1_ReadWriteByte(0XFF);   	//循环读数
	}
	W25QXX_CS=1;
}
/*SPI在一页(0~65535)内写入少于256个字节的数据
在指定地址开始写入最大256字节的数据
pBuffer:数据存储区
WriteAddr:开始写入的地址(24bit)
NumByteToWrite:要写入的字节数(最大256),该数不应该超过该页的剩余字节数!!!*/
void W25QXX_Write_Page(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite){
	u16 i;
	W25QXX_Write_Enable();                  		//SET WEL
	W25QXX_CS=0;                            		//使能器件
	SPI1_ReadWriteByte(W25X_PageProgram);      	//发送写页命令
	SPI1_ReadWriteByte((u8)((WriteAddr)>>16)); 	//发送24bit地址
	SPI1_ReadWriteByte((u8)((WriteAddr)>>8));
	SPI1_ReadWriteByte((u8)WriteAddr);
	for(i=0;i<NumByteToWrite;i++)SPI1_ReadWriteByte(pBuffer[i]);//循环写数
	W25QXX_CS=1;                            	//取消片选
	W25QXX_Wait_Busy();					   		//等待写入结束
}
/*无检验写SPI FLASH
必须确保所写的地址范围内的数据全部为0XFF,否则在非0XFF处写入的数据将失败!
具有自动换页功能
在指定地址开始写入指定长度的数据,但是要确保地址不越界!
pBuffer:数据存储区
WriteAddr:开始写入的地址(24bit)
NumByteToWrite:要写入的字节数(最大65535)
CHECK OK*/
void W25QXX_Write_NoCheck(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite){
	u16 pageremain;
	pageremain=256-WriteAddr%256;		//单页剩余的字节数
	if(NumByteToWrite<=pageremain)pageremain=NumByteToWrite;		//不大于256个字节
	while(1){
		W25QXX_Write_Page(pBuffer,WriteAddr,pageremain);
		if(NumByteToWrite==pageremain)break;		//写入结束了
	 	else{		//NumByteToWrite>pageremain
			pBuffer+=pageremain;
			WriteAddr+=pageremain;
			NumByteToWrite-=pageremain;		//减去已经写入了的字节数
			if(NumByteToWrite>256)pageremain=256;		//一次可以写入256个字节
			else pageremain=NumByteToWrite;		//不够256个字节了
		}
	};
}
/*写SPI FLASH
在指定地址开始写入指定长度的数据
该函数带擦除操作!
pBuffer:数据存储区
WriteAddr:开始写入的地址(24bit)
NumByteToWrite:要写入的字节数(最大65535)*/
u8 W25QXX_BUFFER[4096];
void W25QXX_Write(u8* pBuffer,u32 WriteAddr,u16 NumByteToWrite){
	u32 secpos;
	u16 secoff;
	u16 secremain;
 	u16 i;
	u8 * W25QXX_BUF;
	W25QXX_BUF=W25QXX_BUFFER;
 	secpos=WriteAddr/4096;//扇区地址
	secoff=WriteAddr%4096;//在扇区内的偏移
	secremain=4096-secoff;//扇区剩余空间大小
 	//printf("ad:%X,nb:%X\r\n",WriteAddr,NumByteToWrite);//测试用
 	if(NumByteToWrite<=secremain)secremain=NumByteToWrite;//不大于4096个字节
	while(1){
		W25QXX_Read(W25QXX_BUF,secpos*4096,4096);//读出整个扇区的内容
		for(i=0;i<secremain;i++){//校验数据
			if(W25QXX_BUF[secoff+i]!=0XFF)break;//需要擦除
		}
		if(i<secremain){//需要擦除
			W25QXX_Erase_Sector(secpos);		//擦除这个扇区
			for(i=0;i<secremain;i++){		//复制
				W25QXX_BUF[i+secoff]=pBuffer[i];
			}
			W25QXX_Write_NoCheck(W25QXX_BUF,secpos*4096,4096);		//写入整个扇区

		}else W25QXX_Write_NoCheck(pBuffer,WriteAddr,secremain);		//写已经擦除了的,直接写入扇区剩余区间.
		if(NumByteToWrite==secremain)break;		//写入结束了
		else{		//写入未结束
			secpos++;		//扇区地址增1
			secoff=0;		//偏移位置为0
			pBuffer+=secremain;		//指针偏移
			WriteAddr+=secremain;		//写地址偏移
			NumByteToWrite-=secremain;		//字节数递减
			if(NumByteToWrite>4096)secremain=4096;		//下一个扇区还是写不完
			else secremain=NumByteToWrite;		//下一个扇区可以写完了
		}
	};
}
/*擦除整个芯片
等待时间超长...*/
void W25QXX_Erase_Chip(void){
	W25QXX_Write_Enable();		//SET WEL
	W25QXX_Wait_Busy();
	W25QXX_CS=0;		//使能器件
	SPI1_ReadWriteByte(W25X_ChipErase);		//发送片擦除命令
	W25QXX_CS=1;		//取消片选
	W25QXX_Wait_Busy();		//等待芯片擦除结束
}
/*擦除一个扇区
Dst_Addr:扇区地址 根据实际容量设置
擦除一个山区的最少时间:150ms*/
void W25QXX_Erase_Sector(u32 Dst_Addr){
	//监视falsh擦除情况,测试用
 	printf("fe:%x\r\n",Dst_Addr);
 	Dst_Addr*=4096;
	W25QXX_Write_Enable();		//SET WEL
	W25QXX_Wait_Busy();
	W25QXX_CS=0;		//使能器件
	SPI1_ReadWriteByte(W25X_SectorErase);		//发送扇区擦除指令
	SPI1_ReadWriteByte((u8)((Dst_Addr)>>16));		//发送24bit地址
	SPI1_ReadWriteByte((u8)((Dst_Addr)>>8));
	SPI1_ReadWriteByte((u8)Dst_Addr);
	W25QXX_CS=1;		//取消片选
	W25QXX_Wait_Busy();		//等待擦除完成
}
/*等待空闲*/
void W25QXX_Wait_Busy(void){
	while((W25QXX_ReadSR()&0x01)==0x01);		//等待BUSY位清空
}
/*进入掉电模式*/
void W25QXX_PowerDown(void){
  	W25QXX_CS=0;		//使能器件
    SPI1_ReadWriteByte(W25X_PowerDown);		//发送掉电命令
	W25QXX_CS=1;		//取消片选
    delay_us(3);		//等待TPD
}
/*唤醒*/
void W25QXX_WAKEUP(void){
  	W25QXX_CS=0;		//使能器件
    SPI1_ReadWriteByte(W25X_ReleasePowerDown);		//send W25X_PowerDown command 0xAB
	W25QXX_CS=1;		//取消片选
    delay_us(3);		//等待TRES1
}


