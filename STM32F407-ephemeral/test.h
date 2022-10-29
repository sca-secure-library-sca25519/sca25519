#ifndef __TEST_H
#define __TEST_H

#include <stdint.h>
#include <stdio.h>
#include "crypto/include/randombytes.h"
#include "crypto/include/crypto_scalarmult.h"
#include "crypto/include/fe25519.h"



int test_curve25519_DH(void);
int test_curve25519_DH_TV(void);
int test_curve25519_static(void);

int test_cswap(void);


#endif
