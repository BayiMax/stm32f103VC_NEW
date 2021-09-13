#ifndef __POWER_H
#define __POWER_H

void Power_Get_Init(void);
void IIC_Start(void);
void IIC_Stop(void);
unsigned char IIC_WaitAck(void);
void IIC_SendACK(void);
void IIC_SendNoAck(void);
void IIC_SendByte(unsigned char Byte);
unsigned char IIC_ReceiveByte(void);
void IP5328P_Write(unsigned char reg,unsigned char data);
unsigned char IP5328P_Read(unsigned char reg);
float IP5328P_BatVoltage(void);
float IP5328P_BatCurrent(void);

float IP5328P_BatOCV(void);
unsigned char IP5328P_TypeC_Flag(void);
unsigned char IP5328P_TypeC_Ability(void);
float IP5328P_TypeC_Current(void);
float IP5328P_VIN_Current(void);
float IP5328P_OUT1_Current(void);
float IP5328P_OUT2_Current(void);
float IP5328P_Power(void);
unsigned char IP5328P_SYS_Status(void);
unsigned char IP5328P_KEY_IN(void);
unsigned char IP5328P_VinTypeC_State(void);
unsigned char IP5328P_GHG_State(void);
unsigned char IP5328P_MOS_ON(void);
unsigned char IP5328P_BOOST(void);
unsigned char IP5328P_QC_State(void);
unsigned char IP5328P_DCP_DIG(void);

/****/
void IP5328P_BAT_LOW(unsigned char dat);
void IP5328P_SYS_CTL14(unsigned char chg2bst,unsigned char swclk2,unsigned char swclk1);

#endif /*__POWER_H*/
