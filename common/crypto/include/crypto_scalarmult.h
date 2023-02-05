#ifndef CRYPTO_SCALARMULT_H
#define CRYPTO_SCALARMULT_H

#include <stdint.h>

#include "bigint.h"

#define crypto_scalarmult crypto_scalarmult_curve25519
#define crypto_scalarmult_base crypto_scalarmult_base_curve25519

#define crypto_scalarmult_BYTES 32
#define crypto_scalarmult_SCALARBYTES 32

typedef struct STProtectedStaticKey_curve25519_ {
  uint64_t r;        // Randomization value
  UN_256bitValue k;  // scalar (s * (1/r)) mod sc25519

  // blinding points
  UN_256bitValue Rx;
  UN_256bitValue Ry;
  UN_256bitValue Rz;

  UN_256bitValue Sx;
  UN_256bitValue Sy;
  UN_256bitValue Sz;
} STProtectedStaticKey_curve25519;

// Converted unprotected key to protected static key
int crypto_protectedKeyFromUnprotectedKey_scalarmult_curve25519(
    STProtectedStaticKey_curve25519* k, const uint8_t* s);

void crypto_updateProtectedKey_scalarmult_curve25519(
    STProtectedStaticKey_curve25519* p);

// Protected static scalar multiplication
int crypto_static_scalarmult_curve25519(
    uint8_t* r, const STProtectedStaticKey_curve25519* s, const uint8_t* p);

// Protected static scalar multiplication
int crypto_scalarmult_curve25519(uint8_t* r, const uint8_t* s,
                                 const uint8_t* p);

void update_static_key_curve25519(void);

int crypto_scalarmult_base_curve25519(uint8_t* q, const uint8_t* n);

extern const uint8_t g_basePointCurve25519[32];

#endif
