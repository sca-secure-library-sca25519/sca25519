#include "main.h"

// #include <stdio.h>

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

// cw dependencies
#include "hal.h"
#include "simpleserial.h"

#define DATA_LEN 16  // up to 190 for SS_VER_1_1
#define RESP_LEN 16

const UN_256bitValue unprotected_key = {
    {0x80, 0x65, 0x74, 0xba, 0x61, 0x62, 0xcd, 0x58, 0x49, 0x30, 0x59,
     0x47, 0x36, 0x16, 0x35, 0xb6, 0xe7, 0x7d, 0x7c, 0x7a, 0x83, 0xde,
     0x38, 0xc0, 0x80, 0x74, 0xb8, 0xc9, 0x8f, 0xd4, 0x0a, 0x43}};
#define MAX 100

#if SS_VER == SS_VER_2_1 || SS_VER == SS_VER_2_0
uint8_t sca25519_test(uint8_t cmd, uint8_t scmd, uint8_t dlen, uint8_t* data){
#else
uint8_t sca25519_test(uint8_t* data, uint8_t dlen) {
#endif

  char str[100];
  uint8_t result[32];
  int i;

  uint32_t res, res_dh, res_dhtv;

  res_dh = test_curve25519_DH();

  res_dhtv = test_curve25519_DH_TV();
  res = res_dh + res_dhtv;
  sprintf(str, "Tests output : %lu", res);
  simpleserial_put('r', RESP_LEN, (unsigned char*)str);

  return 0x00;
}

#if SS_VER == SS_VER_2_1 || SS_VER == SS_VER_2_0
uint8_t sca25519_one(uint8_t cmd, uint8_t scmd, uint8_t dlen, uint8_t* data){
#else
uint8_t sca25519_one(uint8_t* data, uint8_t dlen) {
#endif

  char str[100];
  uint8_t result[32];
  int i;

  uint32_t res;
  trigger_high();
  res = test_curve25519_DH_TV_once();
  trigger_low();
  
  sprintf(str, "Sanity check : %lu", res);
  simpleserial_put('r', RESP_LEN, (unsigned char*)str);
  return 0x00;
}

#if SS_VER == SS_VER_2_1 || SS_VER == SS_VER_2_0
uint8_t echo_test(uint8_t cmd, uint8_t scmd, uint8_t dlen, uint8_t* data) {
#else
uint8_t echo_test(uint8_t* data, uint8_t dlen) {
#endif
  trigger_high();
  simpleserial_put('r', RESP_LEN, data);
  trigger_low();
  return 0x00;
}

int main(void) {
  platform_init();
  init_uart();
  trigger_setup();
  simpleserial_init();
  srand(time(0));

  #if SS_VER == SS_VER_2_1 || SS_VER == SS_VER_2_0
    simpleserial_addcmd('a', DATA_LEN, sca25519_test);
    simpleserial_addcmd('b', DATA_LEN, sca25519_one);
    simpleserial_addcmd('e', DATA_LEN, echo_test);
  #else  // SS_VER_1_1, SS_VER_1_0
    simpleserial_addcmd('a', DATA_LEN, sca25519_test);
    simpleserial_addcmd('b', DATA_LEN, sca25519_one);
    simpleserial_addcmd('e', DATA_LEN, echo_test);
  #endif

  while (1) {
    simpleserial_get();
  }

  return 0;
}
