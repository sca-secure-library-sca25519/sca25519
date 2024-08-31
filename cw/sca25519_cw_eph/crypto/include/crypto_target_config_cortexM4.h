#ifndef TARGET_CONFIG_HEADER_CORTEX_M4_
#define TARGET_CONFIG_HEADER_CORTEX_M4_

// We assume, that we are compiling with GCC or with CLANG

#include <stdint.h>

#ifndef NACL_NO_ASM_OPTIMIZATION

#define CRYPTO_HAS_ASM_HSALSA20_BLOCK

// Assembly mpy 256x256 is significantly faster than the C version.
#define CRYPTO_HAS_ASM_MPY_256
#define CRYPTO_HAS_ASM_REDUCE_25519
#define CRYPTO_HAS_ASM_FE25519_MPY121666
#define CRYPTO_HAS_ASM_FE25519_MUL
#define CRYPTO_HAS_ASM_FE25519_SQUARE
#define CRYPTO_HAS_ASM_FE25519_ADD

// Assembly squaring for 256x256 => 512 is considerably faster than the C
// version
#define CRYPTO_HAS_ASM_SQR_256
#define CRYPTO_HAS_ASM_MPY_192
#define CRYPTO_HAS_ASM_SQR_192
#define CRYPTO_HAS_ASM_REDUCE_19119
#define CRYPTO_HAS_ASM_FE19119_MPY132355

#endif

#if defined(__clang__) || defined(__GNUC__)

#define FORCE_INLINE inline __attribute__((__always_inline__))
#define NO_INLINE __attribute__((noinline))

#else

#define FORCE_INLINE
#define NO_INLINE

#endif

#endif  // #ifdef TARGET_CONFIG_HEADER_
