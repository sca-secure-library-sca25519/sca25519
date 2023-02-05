#ifndef CRYPTO_TARGET_CONFIG_HEADER_
#define CRYPTO_TARGET_CONFIG_HEADER_

// #ifdef CRYPTO_USE_USER_TARGET_CONFIG

// #include "crypto_user_target_config.h"

// #elif defined(_MSC_VER)

// #include "crypto_target_config_msc.h"

// #elif defined(CORTEX_M0)

// #include "crypto_target_config_cortexM0.h"

// #elif defined(CORTEX_M4)

#include "crypto_target_config_cortexM4.h"

// #else

// #include <stdint.h>

// #define FORCE_INLINE
// #define NO_INLINE

// #endif

#endif  // #ifdef TARGET_CONFIG_HEADER_
