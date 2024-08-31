#include "../include/randombytes.h"

#include <stdint.h>

#if PLATFORM==CW308_STM32F4
#include "stm32f4xx_hal_rng.h"
RNG_HandleTypeDef hrng;
void randombytes(unsigned char *x, unsigned long long xlen) {
  union {
    unsigned char aschar[4];
    uint32_t asint;
  } random;

  while (xlen > 4) {
    random.asint =  HAL_RNG_GetRandomNumber(&hrng);
    *x++ = random.aschar[0];
    *x++ = random.aschar[1];
    *x++ = random.aschar[2];
    *x++ = random.aschar[3];
    xlen -= 4;
  }
  if (xlen > 0) {
    for (random.asint = HAL_RNG_GetRandomNumber(&hrng); xlen > 0; --xlen) {
      *x++ = random.aschar[xlen - 1];
    }
  }
}
#endif
#if PLATFORM==CWARMLITE
#include <stdlib.h>

uint32_t rand32(void) { return (rand() << 23) ^ (rand() << 11) ^ rand(); }

// #endif

void randombytes(unsigned char* x, unsigned long long len) {
  while (len >= 4) {
    *(uint32_t*)x = rand32();
    x += 4;
    len -= 4;
  }
  if (len == 0) return;
  uint32_t rnd = rand32();
  while (len) {
    *x = rnd;
    rnd >>= 8;
    x++;
    len--;
  }
}
#endif