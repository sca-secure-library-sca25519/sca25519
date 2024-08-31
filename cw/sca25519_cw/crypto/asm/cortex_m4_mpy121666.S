#include "attributesForCortexM4.inc"
	
	.file	"cortex_m0_reduce25519.s"
	
	.text
	.align	2

	.global	fe25519_mpyWith121666_asm
	.code	16
	.thumb_func
	.type	fe25519_mpyWith121666_asm, %function

fe25519_mpyWith121666_asm:
// ######################
// ASM fe25519 mpy 121666, Cortex M4:
// ######################
 // START: fe25519 multiplication with 121666 for M4 (MPY + partial reduce)
 // r0 = result ptr, r1 = input operand ptr.
    push {r4,r14}
    ldr r4,[r1,#28]
    ldr r3,__label_for_immediate_243332
    ldr r2,__label_for_immediate_121665

    .syntax unified
    umull r4,r14,r3,r4

    mov r3,#19
    mul r3,r14
    ldr r14,[r1,#0]
    umaal r14,r3,r2,r14

    str r14,[r0,#0]
    ldr r14,[r1,#4]
    umaal r14,r3,r2,r14

    str r14,[r0,#4]
    ldr r14,[r1,#8]
    umaal r14,r3,r2,r14

    str r14,[r0,#8]
    ldr r14,[r1,#12]
    umaal r14,r3,r2,r14

    str r14,[r0,#12]
    ldr r14,[r1,#16]
    umaal r14,r3,r2,r14

    str r14,[r0,#16]
    ldr r14,[r1,#20]
    umaal r14,r3,r2,r14

    str r14,[r0,#20]
    ldr r14,[r1,#24]
    umaal r14,r3,r2,r14

    str r14,[r0,#24]
    add r3,r3,r4,LSR #1
    str r3,[r0,#28]
    pop {r4,r15}

	.align	2
__label_for_immediate_243332:
	.word 243332
__label_for_immediate_121665:
	.word 121665
	    			
	.size	fe25519_mpyWith121666_asm, .-fe25519_mpyWith121666_asm
		    	
