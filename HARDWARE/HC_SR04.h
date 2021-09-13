#ifndef __HC_SR04_H
#define __HC_SR04_H

#include "main.h"

#if HC_SR04==1
extern unsigned char HC_SR04_Error_Flag;

unsigned char HC_SR04_Init(void);
unsigned short Get_One_distance(void);
unsigned short Get_Distance(void);

#endif

#endif	/*__HC_SR04_H*/

