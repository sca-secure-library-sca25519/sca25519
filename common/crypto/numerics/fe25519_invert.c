#include "../include/fe25519.h"

// Note, that r and x are allowed to overlap!
void fe25519_invert(fe25519* r, const fe25519* x) {
  fe25519 t0;
  fe25519 t1;
  fe25519 t2;

  fe25519_invert_useProvidedScratchBuffers(r, x, &t0, &t1, &t2);
}

// Use this function if optimizing for stack consumption.
// Note, that r and x are allowed to overlap!
void fe25519_invert_useProvidedScratchBuffers(fe25519* r, const fe25519* x,
                                              fe25519* t0, fe25519* t1,
                                              fe25519* t2) {
  fe25519* z11 = r;  // store z11 in r (in order to save one temporary).
  fe25519* z2_10_0 = t1;
  fe25519* z2_50_0 = t2;
  fe25519* z2_100_0 = z2_10_0;

  uint8_t i;

  {
    fe25519* z2 = z2_50_0;

    /* 2 */ fe25519_square(z2, x);
    /* 4 */ fe25519_square(t0, z2);
    /* 8 */ fe25519_square(t0, t0);
    /* 9 */ fe25519_mul(z2_10_0, t0, x);
    /* 11 */ fe25519_mul(z11, z2_10_0, z2);

    // z2 is dead.
  }

  /* 22 */ fe25519_square(t0, z11);
  /* 2^5 - 2^0 = 31 */ fe25519_mul(z2_10_0, t0, z2_10_0);

  /* 2^6 - 2^1 */ fe25519_square(t0, z2_10_0);
  /* 2^7 - 2^2 */ fe25519_square(t0, t0);
  /* 2^8 - 2^3 */ fe25519_square(t0, t0);
  /* 2^9 - 2^4 */ fe25519_square(t0, t0);
  /* 2^10 - 2^5 */ fe25519_square(t0, t0);
  /* 2^10 - 2^0 */ fe25519_mul(z2_10_0, t0, z2_10_0);

  /* 2^11 - 2^1 */ fe25519_square(t0, z2_10_0);

  /* 2^20 - 2^10 */ for (i = 1; i < 10; i++) { fe25519_square(t0, t0); }
  /* 2^20 - 2^0 */ fe25519_mul(z2_50_0, t0, z2_10_0);

  /* 2^21 - 2^1 */ fe25519_square(t0, z2_50_0);

  /* 2^40 - 2^20 */ for (i = 1; i < 20; i++) { fe25519_square(t0, t0); }
  /* 2^40 - 2^0 */ fe25519_mul(t0, t0, z2_50_0);

  /* 2^41 - 2^1 */ fe25519_square(t0, t0);

  /* 2^50 - 2^10 */ for (i = 1; i < 10; i++) { fe25519_square(t0, t0); }
  /* 2^50 - 2^0 */ fe25519_mul(z2_50_0, t0, z2_10_0);

  /* 2^51 - 2^1 */ fe25519_square(t0, z2_50_0);

  /* 2^100 - 2^50 */ for (i = 1; i < 50; i++) { fe25519_square(t0, t0); }
  /* 2^100 - 2^0 */ fe25519_mul(z2_100_0, t0, z2_50_0);

  /* 2^101 - 2^1 */ fe25519_square(t0, z2_100_0);

  /* 2^200 - 2^100 */ for (i = 1; i < 100; i++) { fe25519_square(t0, t0); }
  /* 2^200 - 2^0 */ fe25519_mul(t0, t0, z2_100_0);

  /* 2^250 - 2^50 */ for (i = 0; i < 50; i++) { fe25519_square(t0, t0); }
  /* 2^250 - 2^0 */ fe25519_mul(t0, t0, z2_50_0);

  /* 2^255 - 2^5 */ for (i = 0; i < 5; i++) { fe25519_square(t0, t0); }
  /* 2^255 - 21 */ fe25519_mul(r, t0, z11);
}
