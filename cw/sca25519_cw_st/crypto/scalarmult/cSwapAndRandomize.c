#include <stdint.h>

#define ROTATE16(a) \
  { a = (((uint32_t)a) >> 16) | (a << 16); }

/// swapData is expected to contain the swap status bit in bit #0
/// and fresh random data in bits #1 to #31.
/// pFe1 and pFe2 contain the pointers to the input field elements.
///
/// conditionally swaps the field elements and replaces them
/// with a random multiple.
/// Reduces the result "on-the-fly".
///
void cSwapAndRandomize(uint32_t swapData, uint32_t *pFe1, uint32_t *pFe2,
                       uint32_t randomVal) {
  // we will implement the conditional move half-word wise by
  // generating two values with the swapBit in bit #0 and random
  // data in the upper half-word.

  uint32_t mpyMask1;
  uint32_t mpyMask2;

  // clip the randomized multipliers to 31 bits, such that during
  // reduction the result value may not overflow.
  // also clear bit #15
  uint32_t randomize_mpy = 0x7fff7fff & randomVal;

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
  scaledA = ((uint64_t)randomize_mpy) * outA;
  scaledB = ((uint64_t)randomize_mpy) * outB;

  ROTATE16(in1);
  ROTATE16(in2);

  // conditionally swap the lower 16 bits.
  // the content of the upper 16 bits will be undefined.
  outA = in1 * mpyMask2 + in2 * mpyMask1;
  outB = in1 * mpyMask1 + in2 * mpyMask2;

  outA <<= 16;
  outB <<= 16;

  scaledA += ((uint64_t)randomize_mpy) * outA;
  scaledB += ((uint64_t)randomize_mpy) * outB;

  pFe1[7] = ((uint32_t)scaledA) & 0x7fffffff;
  pFe2[7] = ((uint32_t)scaledB) & 0x7fffffff;

  // reduce the upper bits of the result.
  scaledA >>= 31;
  scaledB >>= 31;
  scaledA *= 19;
  scaledB *= 19;

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

    scaledA += ((uint64_t)randomize_mpy) * outA;
    scaledB += ((uint64_t)randomize_mpy) * outB;

    // handle the upper 16 bits of the two input words i

    // first move the lower bits to the upper half and vice-versa.
    ROTATE16(in1);
    ROTATE16(in2);

    outA = in1 * mpyMask2 + in2 * mpyMask1;
    outB = in1 * mpyMask1 + in2 * mpyMask2;

    outA <<= 16;
    outB <<= 16;

    scaledA += ((uint64_t)randomize_mpy) * outA;
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
