#include "fir.h"

void FirInit(Fir32 *fir) {
  fir->counter = 0;
  for(int i = 0; i < 32; i++) {
    fir->memblock[i] = 0;
  }
}

float FirGet (Fir32 *fir) {
  if (fir->counter == 0)
    return 0;

  int n = 32;
  if (fir->saturated == 0)
    n = fir->counter;

  float sum = 0;

  for(int i = 0; i < n; i++) {
    sum += fir->memblock[i];
  }

  float avg = sum / n;
  return avg;
}

void FirPush (Fir32 *fir, float value) {
  fir->memblock[fir->counter % 32] = value;
  fir->counter++;
  if (fir->counter == 32)
    fir->saturated = 1;
}
