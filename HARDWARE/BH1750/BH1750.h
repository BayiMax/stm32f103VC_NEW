#ifndef __BH1750_H
#define __BH1750_H

#include "main.h"

#if BH1750==1

#define BH1750_Site 0x46    //GND
void BH1750_Init(void);
float Get_BH1750(void);

#endif

#endif	/*__BH1750_H*/

