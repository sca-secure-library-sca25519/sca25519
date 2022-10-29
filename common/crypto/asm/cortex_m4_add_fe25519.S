#include "attributesForCortexM4.inc"

	.code	16
	
	.file	"cortex_m4_sqr_fe25519.S"
	
	.text
	.align	2

		.global	fe25519_add_asm
	.type	fe25519_add_asm, %function

fe25519_add_asm:
    .syntax unified
    push {r4,r5,r6,r14}
    ldr r4,[r1,#(7*4)]
    ldr r3,[r2,#(7*4)]

    mov r14,#1

    mov r5,r3
    umaal r4,r5,r4,r14 // 2 * r4 + r5
    umlal r4,r5,r3,r14 // 2 * r4 + r5 + 1*r3

    mov r3,#19
    mul r3,r5

    ldr r5,[r1,#(0*4)]
    ldr r6,[r2,#(0*4)]

    umaal r5,r6,r14,r3
    str r5,[r0,#(0*4)]

    ldr r3,[r1,#(1*4)]
    ldr r5,[r2,#(1*4)]
    umaal r5,r6,r14,r3
    str r5,[r0,#(1*4)]

    ldr r3,[r1,#(2*4)]
    ldr r5,[r2,#(2*4)]
    umaal r5,r6,r14,r3
    str r5,[r0,#(2*4)]

    ldr r3,[r1,#(3*4)]
    ldr r5,[r2,#(3*4)]
    umaal r5,r6,r14,r3
    str r5,[r0,#(3*4)]

    ldr r3,[r1,#(4*4)]
    ldr r5,[r2,#(4*4)]
    umaal r5,r6,r14,r3
    str r5,[r0,#(4*4)]

    ldr r3,[r1,#(5*4)]
    ldr r5,[r2,#(5*4)]
    umaal r5,r6,r14,r3
    str r5,[r0,#(5*4)]

    ldr r3,[r1,#(6*4)]
    ldr r5,[r2,#(6*4)]
    umaal r5,r6,r14,r3
    str r5,[r0,#(6*4)]

    add r6,r6, r4, LSR #1
    str r6,[r0,#(7*4)]

    pop {r4,r5,r6,r15}

	.size	fe25519_add_asm, .-fe25519_add_asm

