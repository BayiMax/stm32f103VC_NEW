#include "BH1750.h"
#include "main.h"

#if BH1750==1
unsigned char Light_Dat[5];

static void BH1750_IO_Init(void){
	GPIO_InitTypeDef GPIO_InitStruct;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;      //SCL
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStruct);
	GPIOC->BSRR=GPIO_Pin_4;

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5;      //SDA
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStruct);
	GPIOC->BSRR=GPIO_Pin_5;
}

static void BH1750_SDA_OUT(void){
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5;      //SDA
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC,&GPIO_InitStruct);
}
static void BH1750_SDA_IN(void){
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5;      //SDA
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOC,&GPIO_InitStruct);
}

#define BH_SDA(n) n?(GPIOC->BSRR=GPIO_Pin_5):\
                                    (GPIOC->BRR=GPIO_Pin_5)
#define BH_SCL(n) n?(GPIOC->BSRR=GPIO_Pin_4):\
                                    (GPIOC->BRR=GPIO_Pin_4)

#define BH_SDA_Read GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_5)

/*起始信号*/
static void BH1750_IIC_Start(void){
    BH1750_SDA_OUT();
    BH_SDA(1);
    BH_SCL(1);
    delay_us(4);
    BH_SDA(0);
    delay_us(4);
    BH_SCL(0);
}
/*停止信号*/
static void BH1750_IIC_Stop(void){
    BH1750_SDA_OUT();
    BH_SDA(0);
    BH_SCL(1);
    delay_us(4);
    BH_SDA(1);
    delay_us(4);
    
}
/*ack=1:NACK
ack=0:ACK*/
static void BH1750_SendACK(unsigned char ack) {
    BH1750_SDA_OUT();
    if(ack)BH_SDA(1);//NACK 
    else BH_SDA(0); 
    BH_SCL(1);
    delay_us(2);
    BH_SCL(0);
    delay_us(2);
}
/*接受应答信号*/
static unsigned char BH1750_ReadACK(void){
    unsigned char CY=0;
    BH1750_SDA_IN();
    BH_SCL(1);
    delay_us(2);
    CY=BH_SDA_Read;
    BH_SCL(0);
    delay_us(2);
    return CY;
}
/*发送一个字节数据*/
static void BH1750_SendByte(unsigned char Data){
    unsigned char i,bit;
    BH1750_SDA_OUT();
    for(i=0;i<8;i++){
        bit=Data&0x80;
        if(bit) BH_SDA(1);
        else BH_SDA(0);
        Data<<=1;
        BH_SCL(1);
        delay_us(2);
        BH_SCL(0);
        delay_us(2);
    }
    BH1750_ReadACK();
}

/*接收一个字节数据*/
static unsigned char BH1750_ReadByte(void){
    unsigned char i,byte=0;
    BH1750_SDA_IN();
    BH_SDA(1);
    for(i=0;i<8;i++){
        byte<<=1;
        BH_SCL(1);
        delay_us(2);
        if(BH_SDA_Read)byte+=1;
        BH_SCL(0);
        delay_us(2);
    }
    return byte;
}
static void BH1750_WriteCmd(unsigned char Address){
    BH1750_IIC_Start();
    BH1750_SendByte(BH1750_Site);
    BH1750_SendByte(Address);
    BH1750_IIC_Stop();
}

static void BH1750_ReadDat(void){
    unsigned char i;
    BH1750_IIC_Start();
    BH1750_SendByte(BH1750_Site+1);
    for(i=0;i<3;i++){
        Light_Dat[i]=BH1750_ReadByte();
        if(i==3){
            BH1750_SendACK(1);      //1:NACK
        }
        else {
            BH1750_SendACK(0);
        }
    }
    BH1750_IIC_Stop();
    delay_ms(5);
}

void BH1750_Init(void){
    BH1750_IO_Init();
    delay_ms(100);
    BH1750_WriteCmd(0x01);
}

float Get_BH1750(void){
    unsigned short Data;
    float temp1;
    // float temp2;
    BH1750_WriteCmd(0x01);
    BH1750_WriteCmd(0x10);
    delay_ms(5);
    BH1750_ReadDat();
    delay_ms(1);
    printf("LiGht_Dat[0]%d\r\n",Light_Dat[0]);
    printf("LiGht_Dat[1]%d\r\n",Light_Dat[1]);
    Data=Light_Dat[0];
    Data=((Data<<8)+Light_Dat[1]);
    printf("Data:%d\r\n",Data);

    temp1=(float)Data/1.2;
    printf("temp1:%f",temp1);
    // temp2=10*Data/1.2;
    // temp2=(int)temp2%10;        //求余得到小数点后一位
    return temp1;
}


#endif 
