#include "../include/randombytes.h"

#include <stdint.h>
#include <stdlib.h>

uint32_t rand32(void) { return (rand() << 23) ^ (rand() << 11) ^ rand(); }

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
