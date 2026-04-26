#ifndef __BH1750_H
#define __BH1750_H

#include "main.h"

#define BH1750_ADDRESS 0x46 // ADDR pin grounded (7-bit 0x23 << 1)

void BH1750_Init(void);
float BH1750_ReadLux(void);

#endif
