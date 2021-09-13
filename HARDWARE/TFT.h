#ifndef __TFT_H
#define __TFT_H

#include "main.h"

#define USE_HORIZONTAL 2		//���ú�������������ʾ 0��1Ϊ���� 2��3Ϊ����

#define TFT_DC GPIO_Pin_8
#define TFT_RES GPIO_Pin_9
#define TFT_BLK GPIO_Pin_10

#define LCD_RES_Clr()  GPIO_ResetBits(GPIOD,GPIO_Pin_9)		//RES
#define LCD_RES_Set()  GPIO_SetBits(GPIOD,GPIO_Pin_9)

#define LCD_DC_Clr()   GPIO_ResetBits(GPIOD,GPIO_Pin_8)		//DC=RS
#define LCD_DC_Set()   GPIO_SetBits(GPIOD,GPIO_Pin_8)

#define LCD_BLK_Clr()  GPIO_ResetBits(GPIOD,GPIO_Pin_9)		//BLK=LED/*����*/GPIO_SetBits(GPIOG,GPIO_Pin_9)
#define LCD_BLK_Set()  GPIO_SetBits(GPIOD,GPIO_Pin_10)


void LCD_Init(void);
void LCD_Fill(u16 xsta,u16 ysta,u16 xend,u16 yend,u16 color);		//ָ�����������ɫ
void LCD_Color_Fill(u16 xsta,u16 ysta,u16 xend,u16 yend,u16 *color);
void LCD_DrawPoint(u16 x,u16 y,u16 color);		//��ָ��λ�û�һ����
void LCD_DrawLine(u16 x1,u16 y1,u16 x2,u16 y2,u16 color);		//��ָ��λ�û�һ����
void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2,u16 color);		//��ָ��λ�û�һ������
void Draw_Circle(u16 x0,u16 y0,u8 r,u16 color);		//��ָ��λ�û�һ��Բ

void LCD_ShowChinese(u16 x,u16 y,char *s,u16 fc,u16 bc,u8 sizey,u8 mode);		//��ʾ���ִ�
void LCD_ShowChinese12x12(u16 x,u16 y,char *s,u16 fc,u16 bc,u8 sizey,u8 mode);		//��ʾ����12x12����
void LCD_ShowChinese16x16(u16 x,u16 y,char *s,u16 fc,u16 bc,u8 sizey,u8 mode);		//��ʾ����16x16����
void LCD_ShowChinese24x24(u16 x,u16 y,char *s,u16 fc,u16 bc,u8 sizey,u8 mode);		//��ʾ����24x24����
void LCD_ShowChinese32x32(u16 x,u16 y,char *s,u16 fc,u16 bc,u8 sizey,u8 mode);		//��ʾ����32x32����

void LCD_ShowChar(u16 x,u16 y,char num,u16 fc,u16 bc,u8 sizey,u8 mode);		//��ʾһ���ַ�
void LCD_ShowString(u16 x,u16 y,const char *p,u16 fc,u16 bc,u8 sizey,u8 mode);		//��ʾ�ַ���
u32 mypow(u8 m,u8 n);		//����
void LCD_ShowIntNum(u16 x,u16 y,char num,u8 len,u16 fc,u16 bc,u8 sizey);		//��ʾ��������
void LCD_ShowFloatNum1(u16 x,u16 y,float num,u8 len,u16 fc,u16 bc,u8 sizey);		//��ʾ��λС������

void LCD_ShowPicture(u16 x,u16 y,u16 length,u16 width,const u8 pic[]);		//��ʾͼƬ

void LCDDebug(u16 x,u16 y,char *p);


//������ɫ
#define WHITE         	 0xFFFF		//��ɫ
#define BLACK         	 0x0000	  //��ɫ
#define BLUE           	 0x001F  	//��ɫ
#define BRED             0XF81F		//
#define GRED 			 0XFFE0		//
#define GBLUE			 0X07FF		//����ɫ
#define RED           	 0xF800		//��
#define MAGENTA       	 0xF81F
#define GREEN         	 0x07E0		//��
#define CYAN          	 0x7FFF
#define YELLOW        	 0xFFE0		//��
#define PINK			 0XFD79		//��
#define BROWN 			 0XBC40 //��ɫ
#define BRRED 			 0XFC07 //�غ�ɫ
#define GRAY  			 0X8430 //��ɫ
#define DARKBLUE      	 0X01CF	//����ɫ
#define LIGHTBLUE      	 0X7D7C	//ǳ��ɫ
#define GRAYBLUE       	 0X5458 //����ɫ
#define LIGHTGREEN     	 0X841F //ǳ��ɫ
#define LGRAY 			 0XC618 //ǳ��ɫ(PANNEL),���屳��ɫ
#define LGRAYBLUE        0XA651 //ǳ����ɫ(�м����ɫ)
#define LBBLUE           0X2B12 //ǳ����ɫ(ѡ����Ŀ�ķ�ɫ)
#define Rose_red         0xD20F	//õ��

#endif	/*__TFT_H*/



