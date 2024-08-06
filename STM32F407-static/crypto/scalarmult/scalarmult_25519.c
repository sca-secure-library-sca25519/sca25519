#include <limits.h>

#ifdef WITH_PERFORMANCE_BENCHMARKING

#include <stdio.h>

#endif // #ifdef WITH_PERFORMANCE_BENCHMARKING

#include "../../stm32wrapper.h"
#include "../include/crypto_scalarmult.h"
#include "../include/fe25519.h"
#include "../include/randombytes.h"
#include "../include/sc25519.h"
#include "../include/secure_storage.h"

#define MULTIPLICATIVE_CSWAP
#define ITOH_COUNTERMEASURE
#define ITOH_COUNTERMEASURE64

// is the key updatable with respect to randomization - by default no
// static uint8_t updatable = 1;
// static uint8_t fixed = 0;
#define UPDATABLE_STATIC_SCALAR
#define SCALAR_RANDOMIZATION

#ifdef COUNT_CYCLES_EXTRA_SM
unsigned long long globalcount;
#endif

typedef struct _ST_curve25519ladderstepWorkingState {
  // The base point in affine coordinates
  fe25519 x0;

  // The two working points p, q, in projective coordinates. Possibly
  // randomized.
  fe25519 xp;
  fe25519 zp;
  fe25519 xq;
  fe25519 zq;

  UN_256bitValue r;
  UN_256bitValue s;

  int nextScalarBitToProcess;
  uint8_t previousProcessedBit;

} ST_curve25519ladderstepWorkingState;

typedef struct {
  fe25519* x;
  fe25519* y;
  fe25519* z;
} point25519;

extern void curve25519_cswap_asm(ST_curve25519ladderstepWorkingState* state,
                                 uint32_t* b);

#define ROTATE16(a) \
  { a = (((uint32_t)a) >> 16) | (a << 16); }

#define HAS_ASM_cSwapAndRandomize_asm

#ifdef HAS_ASM_cSwapAndRandomize_asm
extern void cSwapAndRandomize_asm(uint32_t swapData, uint32_t* pFe1,
                                  uint32_t* pFe2, uint32_t randomVal);

// let the subsequent code call the assembly function instead of the C function.
#define cSwapAndRandomize cSwapAndRandomize_asm

#else  // #ifdef HAS_ASM_cSwapAndRandomize_asm

/// swapData is expected to contain the swap status bit in bit #0
/// and fresh random data in bits #1 to #31.
/// pFe1 and pFe2 contain the pointers to the input field elements.
///
/// conditionally swaps the field elements and replaces them
/// with a random multiple.
/// Reduces the result "on-the-fly".
///
static void cSwapAndRandomize(uint32_t swapData, uint32_t* pFe1, uint32_t* pFe2,
                              uint32_t randomVal) {
  // we will implement the conditional move half-word wise by
  // generating two values with the swapBit in bit #0 and random
  // data in the upper half-word.

  uint32_t mpyMask1;
  uint32_t mpyMask2;

  // clip the randomized multipliers to 31 bits, such that during
  // reduction the result value may not overflow.
  // also clear bit #15
  uint32_t randomize_mpy = randomVal;

  // Make sure that the randomization multipliers are nonzero.
  randomize_mpy |= 1;

  // generate the "swap-multipliers".
  {
    mpyMask2 = (randomize_mpy ^ swapData) & 0xffff0001;
    mpyMask1 = swapData & 0xffff0001;
  }

  // mpyMask1 and 2 contain in bit #0 the swap status and its complement.
  // bits #1 ... 15 are zero
  // bits #16 to 31 contain random data.

  randomize_mpy &= 0x7fff7fff;

  uint32_t in1, in2;

  // First handle word #7

  in1 = pFe1[7];
  in2 = pFe2[7];

  uint32_t outA, outB;

  // conditionally swap the lower 16 bits.
  // the content of the upper 16 bits will be undefined.
  outA = in1 * mpyMask2 + in2 * mpyMask1;
  outB = in1 * mpyMask1 + in2 * mpyMask2;

  // The least significant 16 bits now contain the swapped content of
  // the most significant half-word of the input field elements
  // The upper part contains random values.

  outA &= 0xffff;
  outB &= 0xffff;

  uint64_t scaledA, scaledB;
  ROTATE16(randomize_mpy);
  scaledA = ((uint64_t)randomize_mpy) * outA;
  ROTATE16(randomize_mpy);
  scaledB = ((uint64_t)randomize_mpy) * outB;

  ROTATE16(in1);
  ROTATE16(in2);

  // conditionally swap the lower 16 bits.
  // the content of the upper 16 bits will be undefined.
  outA = in1 * mpyMask2 + in2 * mpyMask1;
  outB = in1 * mpyMask1 + in2 * mpyMask2;

  outA <<= 16;
  outB <<= 16;

  ROTATE16(randomize_mpy);
  scaledA += ((uint64_t)randomize_mpy) * outA;
  ROTATE16(randomize_mpy);
  scaledB += ((uint64_t)randomize_mpy) * outB;

  pFe1[7] = ((uint32_t)scaledA) & 0x7fffffff;
  pFe2[7] = ((uint32_t)scaledB) & 0x7fffffff;

  // reduce the upper bits of the result.
  scaledA >>= 31;
  scaledB >>= 31;
  scaledA = 19 * ((uint64_t)((uint32_t)scaledA));
  scaledB = 19 * ((uint64_t)((uint32_t)scaledB));

  // now handle the remaining words.
  int i;
  for (i = 0; i < 7; i++) {
    in1 = pFe1[i];
    in2 = pFe2[i];

    // handle the lower 16 bits of the two input words i

    outA = in1 * mpyMask2 + in2 * mpyMask1;
    outB = in1 * mpyMask1 + in2 * mpyMask2;
    outA &= 0xffff;
    outB &= 0xffff;

    ROTATE16(randomize_mpy);
    scaledA += ((uint64_t)randomize_mpy) * outA;
    ROTATE16(randomize_mpy);
    scaledB += ((uint64_t)randomize_mpy) * outB;

    // handle the upper 16 bits of the two input words i

    // first move the lower bits to the upper half and vice-versa.
    ROTATE16(in1);
    ROTATE16(in2);

    outA = in1 * mpyMask2 + in2 * mpyMask1;
    outB = in1 * mpyMask1 + in2 * mpyMask2;

    outA <<= 16;
    outB <<= 16;

    ROTATE16(randomize_mpy);
    scaledA += ((uint64_t)randomize_mpy) * outA;
    ROTATE16(randomize_mpy);
    scaledB += ((uint64_t)randomize_mpy) * outB;

    // write back the results.
    pFe1[i] = (uint32_t)scaledA;
    pFe2[i] = (uint32_t)scaledB;

    scaledA >>= 32;
    scaledB >>= 32;
  }

  // deal with the last carries for word #7
  scaledA += pFe1[7];
  scaledB += pFe2[7];

  pFe1[7] = (uint32_t)scaledA;
  pFe2[7] = (uint32_t)scaledB;
}

#endif  // #ifdef HAS_ASM_cSwapAndRandomize_asm

#define ROTATER(a, cnt) \
  { a = (((uint32_t)a) >> cnt) | (a << (32 - cnt)); }

static void maskScalarBitsWithRandomAndCswap(
    ST_curve25519ladderstepWorkingState* pState, uint32_t wordWithConditionBit,
    uint32_t bitNumber) {
  uint32_t randomDataBuffer[2] = {0, 0};
  randombytes((uint8_t*)randomDataBuffer, sizeof(randomDataBuffer));
  //
  // first combine the scalar bit with a random value which has
  // the bit at the data position cleared
  uint32_t mask = randomDataBuffer[0] & (~(1 << bitNumber));
  wordWithConditionBit ^= mask;

  // Arrange for having the condition bit at bit #0 and random data elsewhere.
  ROTATER(wordWithConditionBit, bitNumber);

  cSwapAndRandomize(wordWithConditionBit, pState->xp.as_uint32_t,
                    pState->xq.as_uint32_t, randomDataBuffer[1]);
  cSwapAndRandomize(wordWithConditionBit, pState->zp.as_uint32_t,
                    pState->zq.as_uint32_t, randomDataBuffer[1]);
}

static void curve25519_ladderstep(ST_curve25519ladderstepWorkingState* pState);

void curve25519_ladderstep(ST_curve25519ladderstepWorkingState* pState) {
  // Implements the "ladd-1987-m-3" differential-addition-and-doubling formulas
  // Source: 1987 Montgomery "Speeding the Pollard and elliptic curve methods of
  // factorization", page 261,
  //         fifth and sixth displays, plus common-subexpression elimination.
  //
  // Notation from the explicit formulas database:
  // (X2,Z2) corresponds to (xp,zp),
  // (X3,Z3) corresponds to (xq,zq)
  // Result (X4,Z4) (X5,Z5) expected in (xp,zp) and (xq,zq)
  //
  // A = X2+Z2; AA = A^2; B = X2-Z2; BB = B^2; E = AA-BB; C = X3+Z3; D = X3-Z3;
  // DA = D*A; CB = C*B; t0 = DA+CB; t1 = t0^2; X5 = Z1*t1; t2 = DA-CB;
  // t3 = t2^2; Z5 = X1*t3; X4 = AA*BB; t4 = a24*E; t5 = BB+t4; Z4 = E*t5 ;
  //
  // Re-Ordered for using less temporaries.

  fe25519 t1, t2;

  fe25519* b1 = &pState->xp;
  fe25519* b2 = &pState->zp;
  fe25519* b3 = &pState->xq;
  fe25519* b4 = &pState->zq;

  fe25519* b5 = &t1;
  fe25519* b6 = &t2;

  fe25519_add(b5, b1, b2);           // A = X2+Z2
  fe25519_sub(b6, b1, b2);           // B = X2-Z2
  fe25519_add(b1, b3, b4);           // C = X3+Z3
  fe25519_sub(b2, b3, b4);           // D = X3-Z3
  fe25519_mul(b3, b2, b5);           // DA= D*A
  fe25519_mul(b2, b1, b6);           // CB= C*B
  fe25519_add(b1, b2, b3);           // T0= DA+CB
  fe25519_sub(b4, b3, b2);           // T2= DA-CB
  fe25519_square(b3, b1);            // X5==T1= T0^2
  fe25519_square(b1, b4);            // T3= t2^2
  fe25519_mul(b4, b1, &pState->x0);  // Z5=X1*t3
  fe25519_square(b1, b5);            // AA=A^2
  fe25519_square(b5, b6);            // BB=B^2
  fe25519_sub(b2, b1, b5);           // E=AA-BB
  fe25519_mul(b1, b5, b1);           // X4= AA*BB
#ifdef CRYPTO_HAS_ASM_COMBINED_MPY121666ADD_FE25519
  fe25519_mpy121666add(b6, b5, b2);
#else
  fe25519_mpyWith121666(b6, b2);  // T4 = a24*E
  fe25519_add(b6, b6, b5);        // T5 = BB + t4
#endif
  fe25519_mul(b2, b6, b2);  // Z4 = E*t5
}

static const fe25519 CON486662 = {
    {0x06, 0x6d, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};

static void curve25519_doublePoint(point25519* R, const point25519* P) {
  // x3 = 2y1z1^2 * (3x1^2z1+2ax1z1^2+z1^3)^2 - 8ay1^3z1^6 - 16x1y1^3z1^5
  // y3 = 4y1^2z1^3 * (3x1+az1) * (3x1^2z1+2ax1z1^2=z1^3) -
  // (3x1^2z1+2ax1z1^2=z1^3)^3 - 8y1^4z1^5
  fe25519 v1, v2, v3, v4, v5, v6, v7;

  fe25519_square(&v6, P->z);
  fe25519_mul(&v6, &v6, P->y);
  fe25519_add(&v6, &v6, &v6);
  fe25519_square(&v1, P->x);
  fe25519_mul(&v1, &v1, P->z);

  fe25519_add(&v2, &v1, &v1);
  fe25519_add(&v1, &v1, &v2);
  fe25519_square(&v2, P->z);
  fe25519_mul(&v2, &v2, P->x);
  fe25519_mul(&v2, &v2, &CON486662);

  fe25519_add(&v2, &v2, &v2);
  fe25519_add(&v1, &v1, &v2);
  fe25519_square(&v2, P->z);
  fe25519_mul(&v2, &v2, P->z);
  fe25519_add(&v1, &v1, &v2);

  fe25519_square(&v2, &v1);
  fe25519_mul(&v6, &v6, &v2);
  fe25519_square(&v3, P->z);
  fe25519_square(&v4, &v3);
  fe25519_mul(&v4, &v4, P->z);

  fe25519_square(&v3, P->y);
  fe25519_mul(&v4, &v4, &v3);
  fe25519_mul(&v4, &v4, P->y);
  fe25519_add(&v4, &v4, &v4);
  fe25519_add(&v4, &v4, &v4);

  fe25519_add(&v4, &v4, &v4);
  fe25519_mul(&v5, &v4, P->z);
  fe25519_mul(&v5, &v5, &CON486662);
  fe25519_sub(&v6, &v6, &v5);
  fe25519_mul(&v5, &v4, P->x);

  fe25519_square(&v7, P->z);
  fe25519_mul(&v7, &v7, P->z);
  fe25519_mul(&v3, &v3, &v7);
  fe25519_add(&v3, &v3, &v3);
  fe25519_add(&v3, &v3, &v3);

  fe25519_mul(&v7, &CON486662, P->z);
  fe25519_add(&v7, &v7, P->x);
  fe25519_add(&v7, &v7, P->x);
  fe25519_add(&v7, &v7, P->x);
  fe25519_mul(&v7, &v7, &v3);

  fe25519_add(&v5, &v5, &v5);
  fe25519_sub(R->x, &v6, &v5);

  fe25519_mul(&v7, &v7, &v1);
  fe25519_mul(&v2, &v2, &v1);
  fe25519_sub(&v7, &v7, &v2);
  fe25519_mul(&v5, &v4, P->y);
  fe25519_sub(R->y, &v7, &v5);

  fe25519_mul(R->z, &v4, P->z);
}

static void curve25519_addPoint(point25519* R, const point25519* P,
                                const point25519* Q) {
  // x3 = (y2z1 - y1z2)^2 * z1z2*(x2z1 - x1z2) - (x2z1-x1z2)^3*(a*z1z2 + x1z2 +
  // x2z1) y3 = ((2*x1z2 + x2z1) + a*z1z2) * (y2z1 - y1z2) * (x1z2 - x1z2)^2 -
  // z1z2*(y2z1 - y1z2)^3 - y1z2*(x2z1 - x1z2)^3 z3 = z1z2*(x2z1 - x1z2)^3

  // 16M + 3S + 10A
  fe25519 y2z1, y1z2, z1z2, x2z1, x1z2;
  fe25519 y2z1my1z2, x2z1mx1z2, x1z2px2z1;
  fe25519 AA, BB, CC, DD;

  fe25519_mul(&y2z1, Q->y, P->z);
  fe25519_mul(&y1z2, P->y, Q->z);
  fe25519_mul(&z1z2, P->z, Q->z);
  fe25519_mul(&x2z1, Q->x, P->z);
  fe25519_mul(&x1z2, P->x, Q->z);

  fe25519_sub(&y2z1my1z2, &y2z1, &y1z2);
  fe25519_sub(&x2z1mx1z2, &x2z1, &x1z2);
  fe25519_add(&x1z2px2z1, &x1z2, &x2z1);

  fe25519_square(&AA, &y2z1my1z2);
  fe25519_mul(&AA, &AA, &x2z1mx1z2);
  fe25519_mul(&AA, &AA, &z1z2);

  fe25519_mul(&BB, &CON486662, &z1z2);
  fe25519_add(&CC, &x1z2px2z1, &x1z2);
  fe25519_add(&CC, &CC, &BB);

  fe25519_square(&DD, &x2z1mx1z2);
  fe25519_mul(&x2z1mx1z2, &DD, &x2z1mx1z2);
  fe25519_mul(&DD, &DD, &y2z1my1z2);
  fe25519_mul(&DD, &DD, &CC);

  fe25519_add(&BB, &BB, &x2z1);
  fe25519_add(&BB, &BB, &x1z2);
  fe25519_mul(&BB, &BB, &x2z1mx1z2);

  fe25519_sub(R->x, &AA, &BB);

  fe25519_square(&AA, &y2z1my1z2);
  fe25519_mul(&AA, &AA, &y2z1my1z2);
  fe25519_mul(&AA, &AA, &z1z2);
  fe25519_sub(R->y, &DD, &AA);
  fe25519_mul(&AA, &y1z2, &x2z1mx1z2);
  fe25519_sub(R->y, R->y, &AA);

  fe25519_mul(R->z, &z1z2, &x2z1mx1z2);
}

// Montgomery y-recovery Algorithm 5 in Montgomery Curves and their arithmetic
static void computeY_curve25519_projective(
    fe25519* y1, fe25519* x1, fe25519* z1,  // (x1,z1) = k*P
    const fe25519* x2, const fe25519* z2,   // (x2,z2)= (k+1)*P
    const fe25519* x, const fe25519* y      // (x,y) = P, z=1
) {
  fe25519 v1, v2, v3, v4;

  fe25519_mul(&v1, x, z1);
  fe25519_add(&v2, x1, &v1);
  fe25519_sub(&v3, x1, &v1);
  fe25519_square(&v3, &v3);
  fe25519_mul(&v3, &v3, x2);

  fe25519_add(&v1, &CON486662, &CON486662);
  fe25519_mul(&v1, &v1, z1);
  fe25519_add(&v2, &v2, &v1);
  fe25519_mul(&v4, x, x1);
  fe25519_add(&v4, &v4, z1);
  fe25519_mul(&v2, &v2, &v4);

  fe25519_mul(&v1, &v1, z1);
  fe25519_sub(&v2, &v2, &v1);
  fe25519_mul(&v2, &v2, z2);
  fe25519_sub(y1, &v2, &v3);
  fe25519_add(&v1, y, y);

  fe25519_mul(&v1, &v1, z1);
  fe25519_mul(&v1, &v1, z2);
  fe25519_mul(x1, &v1, x1);
  fe25519_mul(z1, &v1, z1);
}

static int computeY_curve25519_affine(fe25519* y, const fe25519* x) {
  // y^2 = x^3 + 486662x^2 + x
  fe25519 tmp, x2;

  // x^3
  fe25519_square(&x2, x);
  fe25519_mul(&tmp, &x2, x);
  // 486662x^2
  fe25519_mul(&x2, &x2, &CON486662);

  fe25519_add(&tmp, &tmp, &x2);
  fe25519_add(&tmp, &tmp, x);

  return fe25519_squareroot(y, &tmp);
}

/* This function should be used by use to set his key */
void set_static_key_curve25519(const uint8_t* uRx, const uint8_t* uRy,
                               const uint8_t* uRz, const uint8_t* uSx,
                               const uint8_t* uSy, const uint8_t* uSz,
                               const uint8_t* ustatic_key,
                               const uint8_t* ublindingFactor) {
  uint8_t i;
  for (i = 0; i < 32; i++) {
    Rx.as_uint8_t[i] = uRx[i];
    Ry.as_uint8_t[i] = uRy[i];
    Rz.as_uint8_t[i] = uRz[i];
    Sx.as_uint8_t[i] = uSx[i];
    Sy.as_uint8_t[i] = uSy[i];
    Sz.as_uint8_t[i] = uSz[i];
    static_key.as_uint8_t[i] = ustatic_key[i];
    blindingFactor.as_uint8_t[i] = ublindingFactor[i];
  }
  update_static_key_curve25519();
}

void update_static_key_curve25519() {
  // Update points R, S
  point25519 R, S;
  R.x = &Rx;
  R.y = &Ry;
  R.z = &Rz;
  S.x = &Sx;
  S.y = &Sy;
  S.z = &Sz;
  point25519 R0, S0;
  fe25519 rx0, ry0, rz0, sx0, sy0, sz0;
  R0.x = &rx0;
  R0.y = &ry0;
  R0.z = &rz0;
  S0.x = &sx0;
  S0.y = &sy0;
  S0.z = &sz0;
  fe25519_cpy(R0.x, R.x);
  fe25519_cpy(R0.y, R.y);
  fe25519_cpy(R0.z, R.z);
  fe25519_cpy(S0.x, S.x);
  fe25519_cpy(S0.y, S.y);
  fe25519_cpy(S0.z, S.z);

  uint8_t randbyte;
  randombytes(&randbyte, 1);
  int8_t nextBit = 7;
  while (nextBit >= 0) {
    curve25519_doublePoint(&R, &R);
    curve25519_doublePoint(&S, &S);
    if (randbyte & (1 << nextBit)) {
      curve25519_addPoint(&R, &R, &R0);
      curve25519_addPoint(&S, &S, &S0);
    }
    nextBit--;
  }

#ifdef UPDATABLE_STATIC_SCALAR
  sc25519 newBlindingFactor, newBlindingFactorInverse;
  fe25519_setzero(&newBlindingFactor);

  // Lukasz: new code so newBlindingFactor is not 0
  // unsigned long long;
  do {
    randombytes(newBlindingFactor.as_uint8_t, 8);
  } while (!newBlindingFactor.as_64_bitValue_t[0].as_uint64_t[0]);
  //! fe25519_iszero(&newBlindingFactor)

  fe25519 t, randB;
  UN_512bitValue randVal;
  randombytes(randVal.as_uint8_t, 64);
  fe25519_reduceTo256Bits(&randB, &randVal);
  sc25519_mul(&t, &newBlindingFactor, &randB);
  sc25519_inverse(&t, &t);
  sc25519_mul(&newBlindingFactorInverse, &t, &randB);
  sc25519_mul(&static_key, &static_key, &newBlindingFactorInverse);
  sc25519_mul(&static_key, &static_key, &blindingFactor);
  cpy_256bitvalue(&blindingFactor, &newBlindingFactor);
#endif
}

#if (defined(__clang__) || defined(__GNUC__)) && defined(CORTEX_M4)

#define INCREMENT_BY_NINE(stackVariable)               \
  {                                                    \
    uint32_t scratchReg = 0;                           \
    uint32_t ptrReg = (uint32_t)&stackVariable;        \
    asm volatile(                                      \
        "LDR %[sr], [%[pr]] \n\t"                      \
        "add %[sr], %[sr], #9 \n\t"                    \
        "STR %[sr], [%[pr]]"                           \
        : [ sr ] "+r"(scratchReg), [ pr ] "+r"(ptrReg) \
        :                                              \
        : "memory");                                   \
  }

#define INCREMENT_BY_163(stackVariable)                \
  {                                                    \
    uint32_t scratchReg = 0;                           \
    uint32_t ptrReg = (uint32_t)&stackVariable;        \
    asm volatile(                                      \
        "LDR %[sr], [%[pr]] \n\t"                      \
        "add %[sr], %[sr], #163 \n\t"                  \
        "STR %[sr], [%[pr]]"                           \
        : [ sr ] "+r"(scratchReg), [ pr ] "+r"(ptrReg) \
        :                                              \
        : "memory");                                   \
  }

#else

#define INCREMENT_BY_NINE(a) \
  { a += 9; }

#define INCREMENT_BY_163(a) \
  { a += 163; }

#endif

/// This function implements algorithm 3 from the paper 
/// "SoK: SCA-Secure ECC in software - mission impossible?"
/// (https://tches.iacr.org/index.php/TCHES/article/view/9962)
/// We refer to this paper using "orig." here.
/// We updated this algorithm slightly (especially the order of steps) in: 
/// "An update to the sca25519 library: mitigation card-tearing-based side-channel attacks"
/// (PLACEHOLDER: https://eprint.iacr.org/2024/)
/// The "alg. step" comments refer to this modified version of the algorithm. 
///
/// Comments such as "### alg. step 1 ###" provide to the respective line number of
/// pseudo-code used in the paper.
int crypto_scalarmult_curve25519(uint8_t* r,
                                 /*const uint8_t* s,*/
                                 const uint8_t* p) {
  ST_curve25519ladderstepWorkingState state;
  uint8_t i;
  volatile uint32_t retval = -1;
  volatile uint32_t fid_counter = 0;  // for fault injection detection ### alg. step 1 ###

  // Initialize return value with random bits
  randombytes(r, 32); // ### alg. step 1 (also 1 in orig.) ###

/* Lukasz update:*/
  // update the key and the secret state data
  update_static_key_curve25519(); // ### alg. step 2, orig. 49-51


  // Prepare the scalar within the working state buffer.
  for (i = 0; i < 32; i++) {
    state.s.as_uint8_t[i] = static_key.as_uint8_t[i];
    INCREMENT_BY_NINE(fid_counter);
  } // ### alg. step 3, originally 2 ###

  // Copy the affine x-axis of the base point to the state.
  fe25519_unpack(&state.x0, p); // ### alg. step 15, orig. 14 ###

  // ### alg. step 16, step 15 ###
  fe25519_setone(&state.xq);
  fe25519_setzero(&state.zq);
  fe25519_cpy(&state.xp, &state.x0);
  fe25519_setone(&state.zp);

  INCREMENT_BY_NINE(fid_counter);

  fe25519 yp, y0;

  point25519 P, R;
  P.x = &state.xp;
  P.y = &yp;
  P.z = &state.zp;
  R.x = &Rx;
  R.y = &Ry;
  R.z = &Rz;

  if (computeY_curve25519_affine(&yp, &state.x0) != 0) {
    goto fail;
  } // ### alg. step 4, orig. 3 ###
  
  curve25519_addPoint(&P, &P, &R); // ### alg. step 6, orig. 5 ###

  // Double 3 times before we start ### alg. step 7, orig. 6 ###
  curve25519_doublePoint(&P, &P);
  curve25519_doublePoint(&P, &P);
  curve25519_doublePoint(&P, &P);

  // Randomize scalar multiplicatively
  UN_512bitValue randVal;
#ifdef SCALAR_RANDOMIZATION
  fe25519 t, Rinv, randB;
  fe25519_setzero((fe25519*)&state.r);

  // ### alg. step 8, orig. step 7 ###
  do {
    randombytes(state.r.as_uint8_t, 8);
  } while (!state.r.as_64_bitValue_t[0].as_uint64_t[0]);
  //! fe25519_iszero(&state.r)

  randombytes(randVal.as_uint8_t, 64);
  fe25519_reduceTo256Bits(&randB, &randVal); // ### alg. step 9, orig. 8 ###

  sc25519_mul(&t, &state.r, &randB); // ### alg. step 10, orig. 9 ###
  
  // ### alg. step 11, orig. 10 ###
  sc25519_inverse(&t, &t);
  sc25519_mul(&Rinv, &t, &randB);
  
  sc25519_mul(&state.s, &state.s, &Rinv); // // ### alg. step 12, orig. 11 ###
#else
  fe25519_setone((fe25519*)&state.r);
#endif

  // new re-rand ### alg. step 13, orig. 12 ###
  sc25519_mul(&state.s, &state.s, &blindingFactor);

  INCREMENT_BY_163(fid_counter); // ### alg. step 14, orig. 13 ###

  // Optimize for stack usage when implementing  ### alg. step 14 ###

  fe25519_invert_useProvidedScratchBuffers(&state.zp, &state.zp, &state.xq,
                                           &state.zq, &state.x0);
  fe25519_mul(&state.xp, &state.xp, &state.zp);
  fe25519_reduceCompletely(&state.xp);

  fe25519_mul(&yp, &yp, &state.zp);

  fe25519_cpy(&state.x0, &state.xp);

  //  ### alg. step 17, orig. 16 ###
  // Reinitialize coordinates
  // Prepare the working points within the working state struct.
  randombytes(randVal.as_uint8_t, 64);
  fe25519_reduceTo256Bits(&state.zq, &randVal);
  // paranoia: guarantee that the value is not zero mod p25519
  fe25519_reduceCompletely(&state.zq);
  state.zq.as_uint8_t[31] |= 128;

  fe25519_mul(&state.xq, &state.zq, &state.x0);

  fe25519_setone(&state.xp);
  fe25519_setzero(&state.zp);

  state.nextScalarBitToProcess = 253;  // 252;

  // ### alg. step 18, orig. 17 ###
  // Prepare scalar xor previous bit. Always operate on
  // at least 16 scalar bits together.
  for (i = 7; i >= 1; i--) {
    uint32_t word = state.s.as_uint32_t[i];
    uint32_t previousWord = state.s.as_uint32_t[i - 1];
    uint32_t temp = (word << 1) ^ word;

    state.s.as_uint16_t[2 * i + 1] = temp >> 16;
    word = (word << 16) | (previousWord >> 16);

    word ^= (word << 1);
    state.s.as_uint16_t[2 * i] = word >> 16;
  }
  state.s.as_uint32_t[0] ^= state.s.as_uint32_t[0] << 1;

#ifdef ITOH_COUNTERMEASURE
  // ### alg. step 19, orig. 18 ###
  UN_256bitValue itoh;
  randombytes(itoh.as_uint8_t, 32);
  itoh.as_uint8_t[31] &= 63;  // 31;//15

  UN_256bitValue itohShift;
  cpy_256bitvalue(&itohShift, &itoh);

  // ### alg. step 20, orig. 19 ###
  itohShift.as_uint32_t[7] <<= 1;
  for (i = 7; i >= 1; i--) {
    uint32_t overflow;
    overflow = ((itohShift.as_uint32_t[i - 1] & (1 << 31)) >> 31);
    itohShift.as_uint32_t[i] |= overflow;
    itohShift.as_uint32_t[i - 1] <<= 1;

    state.s.as_uint32_t[i] ^= itoh.as_uint32_t[i];
  }
  state.s.as_uint32_t[0] ^= itoh.as_uint32_t[0];
#endif

#ifdef MULTIPLICATIVE_CSWAP
#else
  state.s.as_uint32_t[7] <<= 2;    // 3;
#ifdef ITOH_COUNTERMEASURE
  itoh.as_uint32_t[7] <<= 2;       // 3;
  itohShift.as_uint32_t[7] <<= 1;  // 3;
#endif
#endif

  INCREMENT_BY_163(fid_counter);

#ifdef ITOH_COUNTERMEASURE
#ifdef MULTIPLICATIVE_CSWAP
  // ### alg. step 22, orig. 21 ###
  maskScalarBitsWithRandomAndCswap(&state, itohShift.as_uint32_t[7], 30);
#else
  curve25519_cswap_asm(&state, &itohShift.as_uint32_t[7]);
#endif
#endif

  // ### alg. step 23, orig. 22 ###
  while (state.nextScalarBitToProcess >= 0) {
    uint8_t limbNo = 0;
    uint8_t bitNo = 0;
#ifdef MULTIPLICATIVE_CSWAP
    {
      limbNo = (uint8_t)(state.nextScalarBitToProcess >> 5);
      bitNo = state.nextScalarBitToProcess & 0x1f;
      // ### alg. step 23, orig. 22 and [XXX]###

      maskScalarBitsWithRandomAndCswap(&state, state.s.as_uint32_t[limbNo],
                                       bitNo);
    }
#else
    {
      limbNo = (uint8_t)(state.nextScalarBitToProcess >> 5);
#ifdef ITOH_COUNTERMEASURE
      uint32_t temp = state.s.as_uint32_t[limbNo] ^ itoh.as_uint32_t[limbNo];
      curve25519_cswap_asm(&state, &temp);
      state.s.as_uint32_t[limbNo] <<= 1;
      itoh.as_uint32_t[limbNo] <<= 1;
#else
      curve25519_cswap_asm(&state, &state.s.as_uint32_t[limbNo]);
#endif
    }
#endif
    if (state.nextScalarBitToProcess >= 1)  // ### alg. step 25, orig. 24
    {
      curve25519_ladderstep(&state); // alg. step 26, orig. 25

      INCREMENT_BY_NINE(fid_counter); // alg. step 28, orig. 27

#ifdef MULTIPLICATIVE_CSWAP
#ifdef ITOH_COUNTERMEASURE
      maskScalarBitsWithRandomAndCswap(&state, itohShift.as_uint32_t[limbNo],
                                       bitNo); // ### alg. step 27, orig. 26
#endif
#else
#ifdef ITOH_COUNTERMEASURE
      curve25519_cswap_asm(&state, &itohShift.as_uint32_t[limbNo]);
#endif
#endif
    }

    state.nextScalarBitToProcess--;
  }

  // ----------------------------------------------------------

#ifdef WITH_PERFORMANCE_BENCHMARKING

#ifdef COUNT_CYCLES_EXTRA_SM
  /*Start cycle count*/
  SCS_DEMCR |= SCS_DEMCR_TRCENA;
  DWT_CYCCNT = 0;
  DWT_CTRL |= DWT_CTRL_CYCCNTENA;
  unsigned int oldcount = DWT_CYCCNT;
#endif

#endif // #ifdef WITH_PERFORMANCE_BENCHMARKING

  // ### alg. step 29, orig. 28
  computeY_curve25519_projective(&y0, &state.xp, &state.zp, &state.xq,
                                 &state.zq, &state.x0, &yp);

  // ### alg. step 30, orig. 29
  // Short scalar multiplication to undo multiplicative scalar blinding
  // Optimize for stack usage.
  fe25519_invert_useProvidedScratchBuffers(&state.zp, &state.zp, &state.xq,
                                           &state.zq, &state.x0);
  fe25519_mul(&state.xp, &state.xp, &state.zp);
  fe25519_reduceCompletely(&state.xp);

  fe25519_mul(&y0, &y0, &state.zp);
  fe25519_cpy(&state.x0, &state.xp);

  // ### alg. step 32, orig. 31
  // Reinitialize coordinates
  // Prepare the working points within the working state struct.
  randombytes(randVal.as_uint8_t, 64);
  fe25519_reduceTo256Bits(&state.zq, &randVal);
  // paranoia: guarantee that the value is not zero mod p25519
  fe25519_reduceCompletely(&state.zq);
  state.zq.as_uint8_t[31] |= 128;
  fe25519_mul(&state.xq, &state.zq, &state.x0);

  // ### alg. step 31, orig. 30
  fe25519_setone(&state.xp);
  fe25519_setzero(&state.zp);

  state.nextScalarBitToProcess = 64;
#if 1
  // Prepare scalar xor previous bit
  // ### alg. step 34 and 35, orig. 33 and 34
  for (i = 2; i >= 1; i--) {
    uint32_t tmp = state.r.as_uint32_t[i] << 1;
    tmp |= state.r.as_uint32_t[i - 1] >> 31;
    state.r.as_uint32_t[i] ^= tmp;
  }

  state.r.as_uint32_t[0] ^= state.r.as_uint32_t[0] << 1;
#else
  // Prepare scalar xor previous bit. Always operate on
  // at least 16 scalar bits together.
  for (i = 2; i >= 1; i--) {
    uint32_t word = state.r.as_uint32_t[i];
    uint32_t previousWord = state.r.as_uint32_t[i - 1];
    uint32_t temp = (word << 1) ^ word;

    state.r.as_uint16_t[2 * i + 1] = temp >> 16;
    word = (word << 16) | (previousWord >> 16);

    word ^= (word << 1);
    state.r.as_uint16_t[2 * i] = word >> 16;
  }
  state.r.as_uint32_t[0] ^= state.r.as_uint32_t[0] << 1;
#endif

#ifdef ITOH_COUNTERMEASURE64
  // ### alg. step 33, orig. 32
  UN_256bitValue itoh64;
  fe25519_setzero((fe25519*)&itoh64);
  randombytes(itoh64.as_uint8_t, 12);
  itoh64.as_uint8_t[2] &= 1;

  UN_256bitValue itoh64Shift;
  cpy_256bitvalue(&itoh64Shift, &itoh64);

  itoh64Shift.as_uint32_t[2] <<= 1;
  for (i = 2; i >= 1; i--) {
    uint32_t overflow;
    overflow = ((itoh64Shift.as_uint32_t[i - 1] & (1 << 31)) >> 31);
    itoh64Shift.as_uint32_t[i] |= overflow;
    itoh64Shift.as_uint32_t[i - 1] <<= 1;

    state.r.as_uint32_t[i] ^= itoh64.as_uint32_t[i];
  }
  state.r.as_uint32_t[0] ^= itoh64.as_uint32_t[0];
#endif

#ifdef MULTIPLICATIVE_CSWAP
#else
  state.r.as_uint32_t[2] <<= 31;
#ifdef ITOH_COUNTERMEASURE64
  itoh64.as_uint32_t[2] <<= 31;
  itoh64Shift.as_uint32_t[2] <<= 30;
#endif
#endif

  INCREMENT_BY_163(fid_counter);

#ifdef ITOH_COUNTERMEASURE64
#ifdef MULTIPLICATIVE_CSWAP
  // ### alg. step 37, orig. 36
  maskScalarBitsWithRandomAndCswap(&state, itoh64Shift.as_uint32_t[2], 1);
#else
  curve25519_cswap_asm(&state, &itoh64Shift.as_uint32_t[2]);
#endif
#endif

  while (state.nextScalarBitToProcess >= 0) // ### alg. step 38, orig. 37
  {
    uint8_t limbNo = 0;
    uint8_t bitNo = 0;

#ifdef MULTIPLICATIVE_CSWAP
    {
      limbNo = (uint8_t)(state.nextScalarBitToProcess >> 5);
      bitNo = state.nextScalarBitToProcess & 0x1f;

      // ### alg. step 39, orig. 38
      maskScalarBitsWithRandomAndCswap(&state, state.r.as_uint32_t[limbNo],
                                       bitNo);
    }
#else
    {
      limbNo = (uint8_t)(state.nextScalarBitToProcess >> 5);
#ifdef ITOH_COUNTERMEASURE64
      uint32_t temp = state.r.as_uint32_t[limbNo] ^ itoh64.as_uint32_t[limbNo];
      curve25519_cswap_asm(&state, &temp);
      state.r.as_uint32_t[limbNo] <<= 1;
      itoh64.as_uint32_t[limbNo] <<= 1;
#else
      curve25519_cswap_asm(&state, &state.r.as_uint32_t[limbNo]);
#endif
    }
#endif

    if (state.nextScalarBitToProcess >= 1) // ### alg. step 40, orig. 39
    {
      curve25519_ladderstep(&state); // ### alg. step 41, orig. 40
      INCREMENT_BY_NINE(fid_counter); // ### alg. step 43, orig. 42

#ifdef MULTIPLICATIVE_CSWAP
#ifdef ITOH_COUNTERMEASURE64
      maskScalarBitsWithRandomAndCswap(&state, itoh64Shift.as_uint32_t[limbNo],
                                       bitNo); // ### alg. step 42, orig. 41
#endif
#else
#ifdef ITOH_COUNTERMEASURE64
      curve25519_cswap_asm(&state, &itoh64Shift.as_uint32_t[limbNo]);
#endif
#endif
    }
    state.nextScalarBitToProcess--;
  }

#ifdef WITH_PERFORMANCE_BENCHMARKING

  /*Comment out cycle counts*/
#ifdef COUNT_CYCLES_EXTRA_SM
  unsigned int newcount = DWT_CYCCNT;
  globalcount += (newcount - oldcount);
#endif

#endif // #ifdef WITH_PERFORMANCE_BENCHMARKING

  // ----------------------------------------------------------
  // Compute y1
  computeY_curve25519_projective(&yp, &state.xp, &state.zp, &state.xq,
                                 &state.zq, &state.x0, &y0); // ### alg. step 43

  point25519 A;
  fe25519 Ay;
  A.x = &Sx;
  A.y = &Ay;
  A.z = &Sz;
  P.x = &state.xp;
  P.y = &yp;
  P.z = &state.zp;
  fe25519_cpy(A.y, &Sy);
  fe25519_neg(A.y, A.y);
  curve25519_addPoint(&P, &P, &A); // ### alg. step 45, orig. 44

  // Optimize for stack usage for ### alg. step 46, orig. 45
  fe25519_invert_useProvidedScratchBuffers(&state.zp, &state.zp, &state.xq,
                                           &state.zq, &state.x0);
  fe25519_mul(&state.xp, &state.xp, &state.zp);
  fe25519_reduceCompletely(&state.xp);
  INCREMENT_BY_163(fid_counter);

  if (fid_counter != (4 * 163 + 350 * 9)) // ### alg. step 48, orig. 47
  {
  fail:
    retval = -1;
    randombytes(state.xp.as_uint8_t, 32); // ### alg. step 49, orig. 48
  } else {
    retval = 0;
  }
  fe25519_pack(r, &state.xp);

/*
  These original update is moved to the beginning
  // update the key and the secret state data
  update_static_key_curve25519(); // ### orig. alg. step 49-51
*/
  return retval;
}

const uint8_t g_basePointCurve25519[32] = {9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

int crypto_scalarmult_base_curve25519(uint8_t* q
                                      // const uint8_t* n
) {
  return crypto_scalarmult_curve25519(q, /*n,*/ g_basePointCurve25519);
}

#ifdef WITH_PERFORMANCE_BENCHMARKING

void cycles_cswap(void);

void cycles_cswap() {
  SCS_DEMCR |= SCS_DEMCR_TRCENA;
  DWT_CYCCNT = 0;
  DWT_CTRL |= DWT_CTRL_CYCCNTENA;

  ST_curve25519ladderstepWorkingState state;
  uint32_t wordwithbit = 10;
  uint32_t bitnum = 10;

  int i;
  unsigned int oldcount = DWT_CYCCNT;
  for (i = 0; i < 1000; i++) {
    maskScalarBitsWithRandomAndCswap(&state, wordwithbit, bitnum);
  }
  unsigned int newcount = DWT_CYCCNT - oldcount;

  char str[100];
  sprintf(str, "Cost cswap: %d", newcount / 1000);
  send_USART_str((unsigned char*)str);
}

#endif // #ifdef WITH_PERFORMANCE_BENCHMARKING

