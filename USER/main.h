#ifndef __MAIN_H
#define __MAIN_H

#define W25Q64_Compile  1
#define FATFS_EN        1
#define malloc_Compile  1
#define SD              1
#define TEXT            1

#define VL53L0X         0
#define MPU6050         0
#define BH1750          0
#define HC_SR04         0

#define IP5328P         1

#include "Arithmetic.h"
#include "stm32f10x.h"
#include "sys.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "ADC.h"

#include "key.h"
#include "led.h"
#include "TFT.h"
#include "InterFace.h"
#include "Modules_IFace.h"
#include "string.h"		
#include "math.h"

#if SD==1
#include "SD_IFace.h"
#include "SD.h"
#include "piclib.h"	
#endif /**/

#include "w25q64.h"

#if malloc_Compile==1
#include "malloc.h"
#endif /**/

#if FATFS_EN==1
#include "ff.h"
#include "exfuns.h"
#include "User_FAT.h"
#endif /**/

#if TEXT==1
#include "fontupd.h"
#include "text.h"	
#endif /**/

#if MPU6050 ==1
#include "mpu6050.h"
#include "Modules_IFace.h"
#endif /**/

#if VL53L0X==1
#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"
#include "vl53l0x_gen.h"
#include "vl53l0x_cali.h"
#include "vl53l0x_it.h"
#include "vl53l0x.h"
#endif 

#if HC_SR04==1
#include "HC_SR04.h"
#endif

#if BH1750==1
#include "BH1750.h"
#endif

#if IP5328P==1
#include "POWER.h"
#endif 

#endif	/*__MAIN_H*/

