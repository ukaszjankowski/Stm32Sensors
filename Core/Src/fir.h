#ifndef __FIR_H
#define __FIR_H

#include "stm32f1xx_hal.h"

typedef struct Fir {
    uint8_t counter;
    float memblock[32];
    uint8_t saturated;
} Fir32;

void FirInit(Fir32 *fir);
float FirGet (Fir32 *fir);
void FirPush (Fir32 *fir, float value);

#endif
