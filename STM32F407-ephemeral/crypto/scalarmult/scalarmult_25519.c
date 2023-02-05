#include "../../stm32wrapper.h"
#include "../include/crypto_scalarmult.h"
#include "../include/fe25519.h"
#include "../include/randombytes.h"
#include "../include/sc25519.h"

#ifdef WITH_PERFORMANCE_BENCHMARKING

#include <stdio.h>

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

extern void curve25519_cswap_asm(ST_curve25519ladderstepWorkingState *state,
                                 uint32_t *b);

// static point for point blinding.
fe25519 Rx = {{0x0f, 0x52, 0x4b, 0xef, 0x7e, 0x12, 0x25, 0x8f, 0xd9, 0xee, 0xfc,
               0xd9, 0xe4, 0x68, 0xb2, 0x1e, 0x54, 0x3d, 0x90, 0xcc, 0x4a, 0x54,
               0xb3, 0x47, 0xf4, 0x8c, 0xc5, 0x96, 0x9f, 0xeb, 0xf4, 0x32}};
fe25519 Ry = {{0xb8, 0x00, 0xed, 0x3e, 0xe5, 0xc6, 0x7f, 0xa7, 0x53, 0x28, 0x30,
               0x59, 0x44, 0xdd, 0x5a, 0x89, 0x66, 0x3f, 0x60, 0xdd, 0xcc, 0x1e,
               0x48, 0xbb, 0xe0, 0xf5, 0x5b, 0x19, 0x25, 0x30, 0x9d, 0x17}};
fe25519 Rz = {{0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
               0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
               0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};

fe25519 Sx = {{0x96, 0xbb, 0x65, 0x32, 0x04, 0x20, 0xa1, 0x8f, 0x53, 0x04, 0x2c,
               0x0e, 0xdd, 0x51, 0xdb, 0xdd, 0xac, 0xf0, 0xb9, 0x65, 0x2f, 0x77,
               0x44, 0x92, 0x64, 0xeb, 0xa9, 0x68, 0x35, 0xad, 0x83, 0x53}};
fe25519 Sy = {{0x36, 0xd0, 0xaa, 0x8c, 0xfe, 0x37, 0x34, 0x3f, 0x51, 0x27, 0x1d,
               0xdb, 0x4b, 0xd4, 0x0f, 0xd3, 0x62, 0x70, 0x80, 0xa0, 0xe5, 0x5c,
               0x48, 0x5f, 0xdf, 0x4a, 0x4f, 0x53, 0xe2, 0x8b, 0x61, 0x32}};
fe25519 Sz = {{0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
               0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
               0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};

static void curve25519_ladderstep(ST_curve25519ladderstepWorkingState *pState);

void curve25519_ladderstep(ST_curve25519ladderstepWorkingState *pState) {
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

  fe25519 *b1 = &pState->xp;
  fe25519 *b2 = &pState->zp;
  fe25519 *b3 = &pState->xq;
  fe25519 *b4 = &pState->zq;

  fe25519 *b5 = &t1;
  fe25519 *b6 = &t2;

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

static void curve25519_doublePointP(
    ST_curve25519ladderstepWorkingState *pState) {
  // Implement the doubling formula "dbl-1987-m-3"
  // from 1987 Montgomery "Speeding the Pollard and elliptic curve methods of
  // factorization", page 261, sixth display, plus common-subexpression
  // elimination.
  //
  // Three operand code:
  // A = X1+Z1
  // AA = A^2
  // B = X1-Z1
  // BB = B^2
  // C = AA-BB
  // X3 = AA*BB
  // t0 = a24*C
  // t1 = BB+t0
  // Z3 = C*t1

  // Double the point input in the state variable "P". Use the State variable
  // "Q" as temporary for storing A, AA and B, BB. Use the same temporary
  // variable for A and AA respectively and B, BB respectively.
  fe25519 *pA = &pState->xq;
  fe25519 *pB = &pState->zq;
  fe25519 *pX = &pState->xp;
  fe25519 *pZ = &pState->zp;
  // A = X1+Z1
  fe25519_add(pA, pX, pZ);
  // AA = A^2
  fe25519_square(pA, pA);
  // B = X1-Z1
  fe25519_sub(pB, pX, pZ);
  // BB = B^2
  fe25519_square(pB, pB);
  // X3 = AA*BB
  fe25519_mul(pX, pA, pB);
  // C = AA-BB
  fe25519_sub(pZ, pA, pB);
  // t0 = a24*C
  fe25519_mpyWith121666(pA, pZ);
  // t1 = BB+t0
  fe25519_add(pB, pA, pB);
  // Z3 = C*t1
  fe25519_mul(pZ, pZ, pB);
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

#define ROTATE16(a) \
  { a = (((uint32_t)a) >> 16) | (a << 16); }

#define HAS_ASM_cSwapAndRandomize_asm
#define MULTIPLICATIVE_CSWAP

#ifdef HAS_ASM_cSwapAndRandomize_asm
extern void cSwapAndRandomize_asm(uint32_t swapData, uint32_t *pFe1,
                                  uint32_t *pFe2, uint32_t randomVal);

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
static void cSwapAndRandomize(uint32_t swapData, uint32_t *pFe1, uint32_t *pFe2,
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

  randomize_mpy &= 0x7fff7fff;

  // mpyMask1 and 2 contain in bit #0 the swap status and its complement.
  // bits #1 ... 15 are zero
  // bits #16 to 31 contain random data.

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
    ST_curve25519ladderstepWorkingState *pState, uint32_t wordWithConditionBit,
    uint32_t bitNumber) {
  uint32_t randomDataBuffer[2] = {0, 0};
  randombytes((uint8_t *)randomDataBuffer, sizeof(randomDataBuffer));
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


/// This function implements algorithm 2 from the paper 
/// "SoK: SCA-Secure ECC in software - mission impossible?"
/// (https://tches.iacr.org/index.php/TCHES/article/view/9962)
///
/// Comments such as "### alg. step 1 ###" provide to the respective line number of
/// pseudo-code used in the paper.
int crypto_scalarmult_curve25519(uint8_t *r, const uint8_t *s,
                                 const uint8_t *p) {
  ST_curve25519ladderstepWorkingState state;
  uint8_t i;
  volatile uint8_t retval = -1;
  // Fault injection detection counter ### alg. step 1 ###
  volatile uint32_t fid_counter = 0;

  // Initialize return value with random bits ### alg. step 2 ###
  randombytes(r, 32);

  // Prepare the scalar within the working state buffer.
  for (i = 0; i < 32; i++) {
    state.s.as_uint8_t[i] = s[i];
  }

  // Copy the affine x-coordinate of the base point to the state.
  fe25519_unpack(&state.x0, p);

  fe25519_setone(&state.xq);
  fe25519_setzero(&state.zq);
  fe25519_cpy(&state.xp, &state.x0);
  fe25519_setone(&state.zp);

  // Clamp scalar ### alg. step 3 ###
  state.s.as_uint8_t[31] &= 127;
  state.s.as_uint8_t[31] |= 64;

  // ### alg. step 4 ###
  shiftRightOne(&state.s);
  shiftRightOne(&state.s);
  shiftRightOne(&state.s);

  // ### alg. step 5 ###
  INCREMENT_BY_163(fid_counter);

  // Double 3 times before we start. ### alg. step 6 ###
  curve25519_doublePointP(&state);
  curve25519_doublePointP(&state);
  curve25519_doublePointP(&state);

  // ### alg. step 7 ###
  INCREMENT_BY_163(fid_counter);

  if (!fe25519_iszero(&state.zp))   // ### alg. step 8 ###
  {
    goto fail; // ### alg. step 9 ###
  }

  // Optimize for stack usage when implementing  ### alg. step 10 ###
  fe25519_invert_useProvidedScratchBuffers(&state.zp, &state.zp, &state.xq,
                                           &state.zq, &state.x0);
  fe25519_mul(&state.xp, &state.xp, &state.zp);
  fe25519_reduceCompletely(&state.xp);

  fe25519_cpy(&state.x0, &state.xp);

  // Reinitialize coordinates
  // Prepare the working points within the working state struct.
  // ### alg. step 12 ###
  UN_512bitValue randVal;
  randombytes(randVal.as_uint8_t, 64);
  fe25519_reduceTo256Bits(&state.zp, &randVal);
  // paranoia: guarantee that the value is not zero mod p25519
  fe25519_reduceCompletely(&state.zp);
  state.zp.as_uint8_t[31] |= 128;

  fe25519_mul(&state.xp, &state.zp, &state.x0);  // ### alg. step 12 ###

  fe25519_setone(&state.xq);  // ### alg. step 11 ###
  fe25519_setzero(&state.zq);

  // Perform ladderstep for first bit that is always 1
  curve25519_ladderstep(&state);

  state.nextScalarBitToProcess = 251;

  // Prepare scalar xor previous bit. Always operate on
  // at least 16 scalar bits together. ### alg. step 13 ###
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

#ifdef MULTIPLICATIVE_CSWAP
#else
  state.s.as_uint32_t[7] <<= 4;
#endif

  INCREMENT_BY_163(fid_counter); // ### alg. step 14 ###

  while (state.nextScalarBitToProcess >= 0) // ### alg. step 15 ###
  {
#ifdef MULTIPLICATIVE_CSWAP
    {
      uint8_t limbNo = (uint8_t)(state.nextScalarBitToProcess >> 5);
      uint8_t bitNo = state.nextScalarBitToProcess & 0x1f;
      maskScalarBitsWithRandomAndCswap(&state, state.s.as_uint32_t[limbNo],
                                       bitNo);  // ### alg. step 16 and 19 ###.
    }
#else
    {
      uint8_t limbNo = (uint8_t)(state.nextScalarBitToProcess >> 5);
      curve25519_cswap_asm(&state, &state.s.as_uint32_t[limbNo]);
    }
#endif
    if (state.nextScalarBitToProcess >= 1) {
      curve25519_ladderstep(&state);  // ### alg. step 17 ###

      INCREMENT_BY_NINE(fid_counter);  // ### alg. step 18 ###
    }

    state.nextScalarBitToProcess--;
  }

  // Optimize for stack usage when implementing ### alg. step 20 ###
  fe25519_invert_useProvidedScratchBuffers(&state.zp, &state.zp, &state.xq,
                                           &state.zq, &state.x0);
  fe25519_mul(&state.xp, &state.xp, &state.zp);
  fe25519_reduceCompletely(&state.xp);

  INCREMENT_BY_163(fid_counter); // ### alg. step 21 ###

  // ### alg. step 22 ###
  if (fid_counter != (163 * 4 + 251 * 9)) {
  fail:
    retval = -1;
    randombytes(state.xp.as_uint8_t, 32);  // ### alg. step 23 ###
  } else {
    retval = 0;
  }
  fe25519_pack(r, &state.xp);
  return retval;
}

const uint8_t g_basePointCurve25519[32] = {9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                           0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

int crypto_scalarmult_base_curve25519(uint8_t *q, const uint8_t *n) {
  return crypto_scalarmult_curve25519(q, n, g_basePointCurve25519);
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
  sprintf(str, "Cost cswap: %d", (unsigned)(newcount / 1000));
  send_USART_str((unsigned char *)str);
}

#endif //#ifdef WITH_PERFORMANCE_BENCHMARKING

