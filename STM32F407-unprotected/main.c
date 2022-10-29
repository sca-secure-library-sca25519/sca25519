#include "stm32wrapper.h"
#include <stdio.h>
#include "main.h"

const UN_256bitValue unprotected_key = {{   0x80, 0x65, 0x74, 0xba, 0x61, 0x62, 0xcd, 0x58,
                                            0x49, 0x30, 0x59, 0x47, 0x36, 0x16, 0x35, 0xb6,
                                            0xe7, 0x7d, 0x7c, 0x7a, 0x83, 0xde, 0x38, 0xc0,
                                            0x80, 0x74, 0xb8, 0xc9, 0x8f, 0xd4, 0x0a, 0x43}};
#define MAX 100

int main(void)
{
    clock_setup();
    gpio_setup();
    usart_setup(115200);
    rng_enable();
    char str[100];

    send_USART_str((unsigned char*)"Program started."); 

    uint8_t result[32];
    int i;
    unsigned int oldcount;
    unsigned long long newcount = 0;
    SCS_DEMCR |= SCS_DEMCR_TRCENA;
    DWT_CYCCNT = 0;
    DWT_CTRL |= DWT_CTRL_CYCCNTENA;
    for (i = 0; i < MAX; i++) {
        oldcount = DWT_CYCCNT;
        crypto_scalarmult_base_curve25519(result, unprotected_key.as_uint8_t);
        newcount += (DWT_CYCCNT - oldcount);
    }
    //unsigned int newcount = DWT_CYCCNT-oldcount;

    sprintf(str, "Cost of scalarmult: %d", newcount / MAX);
    send_USART_str((unsigned char*) str);

    uint32_t res;

    send_USART_str((unsigned char*)"Test scalarmult!");

    res = test_curve25519_DH();
    sprintf(str, "Test DH(0 correct): %lu", res);
    send_USART_str((unsigned char*) str);

    res = test_curve25519_DH_TV();
    sprintf(str, "Test DH TV(0 correct): %lu", res);
    send_USART_str((unsigned char*) str);

    send_USART_str((unsigned char*)"Done!");

    while(1);

    return 0;
}
