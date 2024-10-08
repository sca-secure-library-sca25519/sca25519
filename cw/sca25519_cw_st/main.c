#include "main.h"

#include <stdio.h>

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

// cw dependencies
#include "hal.h"
#include "simpleserial.h"

#define DATA_LEN 16  // up to 190 for SS_VER_1_1
#define RESP_LEN 16

// blinded key prepared by user
UN_256bitValue ustatic_key = {{0x1b, 0xc7, 0x9e, 0xf8, 0xe8, 0x24, 0xb2, 0x7a,
                               0x9d, 0xc5, 0x9b, 0x41, 0x63, 0x5a, 0x9c, 0x9d,
                               0xab, 0x9e, 0xd3, 0xaf, 0xa6, 0x0f, 0x34, 0xc5,
                               0xa5, 0xe0, 0xa1, 0xc4, 0x74, 0x29, 0x3a, 0x0d}};
UN_256bitValue ublindingFactor = {
    {0x23, 0x09, 0xef, 0x5b, 0x32, 0x46, 0xdb, 0x2f, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
// static point for point blinding - prepared by user
UN_256bitValue uRx = {{0x0f, 0x52, 0x4b, 0xef, 0x7e, 0x12, 0x25, 0x8f,
                       0xd9, 0xee, 0xfc, 0xd9, 0xe4, 0x68, 0xb2, 0x1e,
                       0x54, 0x3d, 0x90, 0xcc, 0x4a, 0x54, 0xb3, 0x47,
                       0xf4, 0x8c, 0xc5, 0x96, 0x9f, 0xeb, 0xf4, 0x32}};
UN_256bitValue uRy = {{0xb8, 0x00, 0xed, 0x3e, 0xe5, 0xc6, 0x7f, 0xa7,
                       0x53, 0x28, 0x30, 0x59, 0x44, 0xdd, 0x5a, 0x89,
                       0x66, 0x3f, 0x60, 0xdd, 0xcc, 0x1e, 0x48, 0xbb,
                       0xe0, 0xf5, 0x5b, 0x19, 0x25, 0x30, 0x9d, 0x17}};
UN_256bitValue uRz = {{0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};

UN_256bitValue uSx = {{0x96, 0xbb, 0x65, 0x32, 0x04, 0x20, 0xa1, 0x8f,
                       0x53, 0x04, 0x2c, 0x0e, 0xdd, 0x51, 0xdb, 0xdd,
                       0xac, 0xf0, 0xb9, 0x65, 0x2f, 0x77, 0x44, 0x92,
                       0x64, 0xeb, 0xa9, 0x68, 0x35, 0xad, 0x83, 0x53}};
UN_256bitValue uSy = {{0x36, 0xd0, 0xaa, 0x8c, 0xfe, 0x37, 0x34, 0x3f,
                       0x51, 0x27, 0x1d, 0xdb, 0x4b, 0xd4, 0x0f, 0xd3,
                       0x62, 0x70, 0x80, 0xa0, 0xe5, 0x5c, 0x48, 0x5f,
                       0xdf, 0x4a, 0x4f, 0x53, 0xe2, 0x8b, 0x61, 0x32}};
UN_256bitValue uSz = {{0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                       0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};

#if SS_VER == SS_VER_2_1 || SS_VER == SS_VER_2_0
uint8_t sca25519_test(uint8_t cmd, uint8_t scmd, uint8_t dlen, uint8_t* data){
#else
uint8_t sca25519_test(uint8_t* data, uint8_t dlen) {
#endif

  char str[100];
  uint8_t result[32];
  int i;

  uint32_t res, res_dh, res_dhtv;

  res_dh = test_point();

  res_dhtv = test_curve25519_static();
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
  res = test_curve25519_static_once();
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
