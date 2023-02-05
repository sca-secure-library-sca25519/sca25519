#include "main.h"

#include <stdio.h>

#include "stm32wrapper.h"

// Original static_key
const UN_256bitValue ephermeral_key = {
    {0x80, 0x65, 0x74, 0xba, 0x61, 0x62, 0xcd, 0x58, 0x49, 0x30, 0x59,
     0x47, 0x36, 0x16, 0x35, 0xb6, 0xe7, 0x7d, 0x7c, 0x7a, 0x83, 0xde,
     0x38, 0xc0, 0x80, 0x74, 0xb8, 0xc9, 0x8f, 0xd4, 0x0a, 0x43}};

#define MAX 1000

int main(void) {
  clock_setup();
  gpio_setup();
  usart_setup(115200);
  rng_enable();
  char str[100];

  send_USART_str((unsigned char *)"Program started.");

  SCS_DEMCR |= SCS_DEMCR_TRCENA;
  DWT_CYCCNT = 0;
  DWT_CTRL |= DWT_CTRL_CYCCNTENA;
  uint8_t result[32];
  int i;
  unsigned long long totalcountNumber = 0;
  unsigned int oldcount, newcount;
  for (i = 0; i < MAX; i++) {
    oldcount = DWT_CYCCNT;
    crypto_scalarmult_base_curve25519(result, ephermeral_key.as_uint8_t);
    newcount = DWT_CYCCNT;
    if (newcount < oldcount) {
      sprintf(str, "Clock Overflown");
      send_USART_str((unsigned char *)str);
    } else {
      totalcountNumber += ((long long)newcount - (long long)oldcount);
    }
  }
  sprintf(str, "Cost: %d", (unsigned)(totalcountNumber / MAX));
  send_USART_str((unsigned char *)str);

  void cycles_cswap(void);
  cycles_cswap();

  uint32_t res;

  send_USART_str((unsigned char *)"Test scalarmult!");

  res = test_curve25519_DH();
  sprintf(str, "Test DH(0 correct): %lu", res);
  send_USART_str((unsigned char *)str);

  res = test_curve25519_DH_TV();
  sprintf(str, "Test DH TV(0 correct): %lu", res);
  send_USART_str((unsigned char *)str);

  send_USART_str((unsigned char *)"Done!");

  while (1)
    ;

  return 0;
}
