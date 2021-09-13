#ifndef __TFT_H
#define __TFT_H

#include "main.h"

#define USE_HORIZONTAL 2		//设置横屏或者竖屏显示 0或1为竖屏 2或3为横屏

#define TFT_DC GPIO_Pin_8
#define TFT_RES GPIO_Pin_9
#define TFT_BLK GPIO_Pin_10

#define LCD_RES_Clr()  GPIO_ResetBits(GPIOD,GPIO_Pin_9)		//RES
#define LCD_RES_Set()  GPIO_SetBits(GPIOD,GPIO_Pin_9)

#define LCD_DC_Clr()   GPIO_ResetBits(GPIOD,GPIO_Pin_8)		//DC=RS
#define LCD_DC_Set()   GPIO_SetBits(GPIOD,GPIO_Pin_8)

#define LCD_BLK_Clr()  GPIO_ResetBits(GPIOD,GPIO_Pin_9)		//BLK=LED/*背光*/GPIO_SetBits(GPIOG,GPIO_Pin_9)
#define LCD_BLK_Set()  GPIO_SetBits(GPIOD,GPIO_Pin_10)


void LCD_Init(void);
void LCD_Fill(u16 xsta,u16 ysta,u16 xend,u16 yend,u16 color);		//指定区域填充颜色
void LCD_Color_Fill(u16 xsta,u16 ysta,u16 xend,u16 yend,u16 *color);
void LCD_DrawPoint(u16 x,u16 y,u16 color);		//在指定位置画一个点
void LCD_DrawLine(u16 x1,u16 y1,u16 x2,u16 y2,u16 color);		//在指定位置画一条线
void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2,u16 color);		//在指定位置画一个矩形
void Draw_Circle(u16 x0,u16 y0,u8 r,u16 color);		//在指定位置画一个圆

void LCD_ShowChinese(u16 x,u16 y,char *s,u16 fc,u16 bc,u8 sizey,u8 mode);		//显示汉字串
void LCD_ShowChinese12x12(u16 x,u16 y,char *s,u16 fc,u16 bc,u8 sizey,u8 mode);		//显示单个12x12汉字
void LCD_ShowChinese16x16(u16 x,u16 y,char *s,u16 fc,u16 bc,u8 sizey,u8 mode);		//显示单个16x16汉字
void LCD_ShowChinese24x24(u16 x,u16 y,char *s,u16 fc,u16 bc,u8 sizey,u8 mode);		//显示单个24x24汉字
void LCD_ShowChinese32x32(u16 x,u16 y,char *s,u16 fc,u16 bc,u8 sizey,u8 mode);		//显示单个32x32汉字

void LCD_ShowChar(u16 x,u16 y,char num,u16 fc,u16 bc,u8 sizey,u8 mode);		//显示一个字符
void LCD_ShowString(u16 x,u16 y,const char *p,u16 fc,u16 bc,u8 sizey,u8 mode);		//显示字符串
u32 mypow(u8 m,u8 n);		//求幂
void LCD_ShowIntNum(u16 x,u16 y,char num,u8 len,u16 fc,u16 bc,u8 sizey);		//显示整数变量
void LCD_ShowFloatNum1(u16 x,u16 y,float num,u8 len,u16 fc,u16 bc,u8 sizey);		//显示两位小数变量

void LCD_ShowPicture(u16 x,u16 y,u16 length,u16 width,const u8 pic[]);		//显示图片

void LCDDebug(u16 x,u16 y,char *p);


//画笔颜色
#define WHITE         	 0xFFFF		//白色
#define BLACK         	 0x0000	  //黑色
#define BLUE           	 0x001F  	//蓝色
#define BRED             0XF81F		//
#define GRED 			 0XFFE0		//
#define GBLUE			 0X07FF		//天青色
#define RED           	 0xF800		//红
#define MAGENTA       	 0xF81F
#define GREEN         	 0x07E0		//绿
#define CYAN          	 0x7FFF
#define YELLOW        	 0xFFE0		//黄
#define PINK			 0XFD79		//粉
#define BROWN 			 0XBC40 //棕色
#define BRRED 			 0XFC07 //棕红色
#define GRAY  			 0X8430 //灰色
#define DARKBLUE      	 0X01CF	//深蓝色
#define LIGHTBLUE      	 0X7D7C	//浅蓝色
#define GRAYBLUE       	 0X5458 //灰蓝色
#define LIGHTGREEN     	 0X841F //浅绿色
#define LGRAY 			 0XC618 //浅灰色(PANNEL),窗体背景色
#define LGRAYBLUE        0XA651 //浅灰蓝色(中间层颜色)
#define LBBLUE           0X2B12 //浅棕蓝色(选择条目的反色)
#define Rose_red         0xD20F	//玫红

#endif	/*__TFT_H*/



