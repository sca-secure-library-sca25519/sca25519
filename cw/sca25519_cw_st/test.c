#include "test.h"

int test_curve25519_static() {
  uint8_t pk_A[32];

  uint8_t correct_res[] = {0xd3, 0x7e, 0x13, 0xaa, 0xb8, 0x86, 0xca, 0xff,
                           0x84, 0x90, 0x92, 0x60, 0x60, 0xc8, 0x51, 0x55,
                           0x06, 0xe5, 0x69, 0x3e, 0xb9, 0xac, 0x2e, 0x4e,
                           0x14, 0xe5, 0xaf, 0x19, 0x4f, 0x6f, 0x67, 0x6e};

  int i;
  volatile int correct = 0;
  correct |=
      crypto_scalarmult_base_curve25519(pk_A /*, static_key.as_uint8_t*/);

  for (i = 0; i < 32; i++) {
    if (pk_A[i] != correct_res[i]) {
      correct |= 1;
      break;
    }
  }
  correct |=
      crypto_scalarmult_base_curve25519(pk_A /*, static_key.as_uint8_t*/);
  for (i = 0; i < 32; i++) {
    if (pk_A[i] != correct_res[i]) {
      correct |= 2;
      break;
    }
  }

  return correct;
}
int test_point(void);

int test_point() {
  uint8_t input_point[32];
  uint8_t out_point1[32], out_point2[32];
  int i, j;
  int retval = 0;
  int out = 0;
  for (i = 0; i < 10; i++) {
    randombytes((uint8_t *)&input_point, 32);
    out = crypto_scalarmult_curve25519(out_point1, input_point);
    crypto_scalarmult_curve25519(out_point2, input_point);
    if (out != 0) continue;
    for (j = 0; j < 32; j++) {
      if (out_point1[j] != out_point2[j]) {
        retval |= i;
      }
    }
  }

  return retval;
}

int test_curve25519_static_once() {
  uint8_t pk_A[32];

  uint8_t correct_res[] = {0xd3, 0x7e, 0x13, 0xaa, 0xb8, 0x86, 0xca, 0xff,
                           0x84, 0x90, 0x92, 0x60, 0x60, 0xc8, 0x51, 0x55,
                           0x06, 0xe5, 0x69, 0x3e, 0xb9, 0xac, 0x2e, 0x4e,
                           0x14, 0xe5, 0xaf, 0x19, 0x4f, 0x6f, 0x67, 0x6e};

  int i;
  volatile int correct = 0;
  correct |=
      crypto_scalarmult_base_curve25519(pk_A /*, static_key.as_uint8_t*/);

  for (i = 0; i < 32; i++) {
    if (pk_A[i] != correct_res[i]) {
      correct |= 1;
      break;
    }
  }

  return correct;
}
