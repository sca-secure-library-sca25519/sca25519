#include "test.h"

static void flipEndian(uint8_t *output, const uint8_t *input) {
  int i;
  for (i = 0; i < 32; i++) {
    output[i] = input[31 - i];
  }
}

// Test ECDH with random public keys
// Return 0 if shared secret is equal, 1 otherwise
int test_curve25519_DH() {
  // const char insecure_dummy_rng_seed[] = "Insecure Dummy Seed";
  // randombytes_reseed
  // (insecure_dummy_rng_seed,sizeof(insecure_dummy_rng_seed));

  uint8_t sk_A[32];
  uint8_t sk_B[32];
  uint8_t pk_A[32];
  uint8_t pk_B[32];
  uint8_t ss_A[32];
  uint8_t ss_B[32];

  randombytes(sk_A, 32);
  randombytes(sk_B, 32);

  crypto_scalarmult_base_curve25519(pk_A, sk_A);
  crypto_scalarmult_base_curve25519(pk_B, sk_B);

  crypto_scalarmult_curve25519(ss_A, sk_A, pk_B);
  crypto_scalarmult_curve25519(ss_B, sk_B, pk_A);

  int i;
  volatile int correct = 0;
  for (i = 0; i < 32; i++) {
    if (ss_A[i] != ss_B[i]) {
      correct = 1;
      break;
    }
  }
  return correct;
}

int test_curve25519_DH_TV() {
  int i;
  int result = 0;
  uint8_t insk_A[] = {0x6A, 0x2C, 0xB9, 0x1D, 0xA5, 0xFB, 0x77, 0xB1,
                      0x2A, 0x99, 0xC0, 0xEB, 0x87, 0x2F, 0x4C, 0xDF,
                      0x45, 0x66, 0xB2, 0x51, 0x72, 0xC1, 0x16, 0x3C,
                      0x7D, 0xA5, 0x18, 0x73, 0x0A, 0x6D, 0x07, 0x70};
  uint8_t insk_B[] = {0x6B, 0xE0, 0x88, 0xFF, 0x27, 0x8B, 0x2F, 0x1C,
                      0xFD, 0xB6, 0x18, 0x26, 0x29, 0xB1, 0x3B, 0x6F,
                      0xE6, 0x0E, 0x80, 0x83, 0x8B, 0x7F, 0xE1, 0x79,
                      0x4B, 0x8A, 0x4A, 0x62, 0x7E, 0x08, 0xAB, 0x58};
  uint8_t sk_A[32];
  uint8_t sk_B[32];
  flipEndian(sk_A, insk_A);
  flipEndian(sk_B, insk_B);

  uint8_t pk_A[32];
  uint8_t pk_B[32];
  uint8_t ss_A[32];
  uint8_t ss_B[32];

  uint8_t correct_pk_A[] = {0x85, 0x20, 0xF0, 0x09, 0x89, 0x30, 0xA7, 0x54,
                            0x74, 0x8B, 0x7D, 0xDC, 0xB4, 0x3E, 0xF7, 0x5A,
                            0x0D, 0xBF, 0x3A, 0x0D, 0x26, 0x38, 0x1A, 0xF4,
                            0xEB, 0xA4, 0xA9, 0x8E, 0xAA, 0x9B, 0x4E, 0x6A};
  uint8_t correct_pk_B[] = {0xDE, 0x9E, 0xDB, 0x7D, 0x7B, 0x7D, 0xC1, 0xB4,
                            0xD3, 0x5B, 0x61, 0xC2, 0xEC, 0xE4, 0x35, 0x37,
                            0x3F, 0x83, 0x43, 0xC8, 0x5B, 0x78, 0x67, 0x4D,
                            0xAD, 0xFC, 0x7E, 0x14, 0x6F, 0x88, 0x2B, 0x4F};
  uint8_t correct_ss[] = {0x4A, 0x5D, 0x9D, 0x5B, 0xA4, 0xCE, 0x2D, 0xE1,
                          0x72, 0x8E, 0x3B, 0xF4, 0x80, 0x35, 0x0F, 0x25,
                          0xE0, 0x7E, 0x21, 0xC9, 0x47, 0xD1, 0x9E, 0x33,
                          0x76, 0xF0, 0x9B, 0x3C, 0x1E, 0x16, 0x17, 0x42};

  crypto_scalarmult_base_curve25519(pk_A, sk_A);
  for (i = 0; i < 32; i++) {
    if (pk_A[i] != correct_pk_A[i]) {
      result |= 1;
      break;
    }
  }

  crypto_scalarmult_base_curve25519(pk_B, sk_B);
  for (i = 0; i < 32; i++) {
    if (pk_B[i] != correct_pk_B[i]) {
      result |= 2;
      break;
    }
  }

  crypto_scalarmult_curve25519(ss_A, sk_A, pk_B);
  for (i = 0; i < 32; i++) {
    if (ss_A[i] != correct_ss[i]) {
      result |= 4;
      break;
    }
  }

  crypto_scalarmult_curve25519(ss_B, sk_B, pk_A);
  for (i = 0; i < 32; i++) {
    if (ss_B[i] != correct_ss[i]) {
      result |= 8;
      break;
    }
  }
  return result;
}
