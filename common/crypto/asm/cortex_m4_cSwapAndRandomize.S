//#define SWAP_PROJECTIVE_RANDOMIZATION_VALUE 1

#include "attributesForCortexM4.inc"

	.file	"cortex_m4_cSwapAndRandomize.S"
	.text
	.align	1
	.p2align 2,,3
	.global	cSwapAndRandomize_asm
	.syntax unified
	.thumb
	.thumb_func
	.fpu softvfp
	.type	cSwapAndRandomize_asm, %function
cSwapAndRandomize_asm:

#if 0
        // unprotected, variable time ONLY FOR TESTING !!!
	push	{r1, r4, r5, r6, r7, r8, r9, r10, fp, lr}
        // r0 : swap bit and random data.
        // r1 : pFe1
        // r2 : pFe2
        // r3 : randomization mask

        ands  r0,r0,#1
        beq .noSwap
        add r5,r1,#32
.loopStart:
        ldr r3,[r1,#0]
        ldr r4,[r2,#0] 
        str r3,[r2,#0]
        str r4,[r1,#0] 

        add r1,r1,#4
        add r2,r2,#4

        cmp r1,r5
        bne .loopStart
.noSwap:   

        // use dummy pop for r1 for the single stack slot needed.
	pop	{r1, r4, r5, r6, r7, r8, r9, r10, fp, pc}

#else

#define ROTATE_RANDOM_MASK ror r3,r3,#16

        // protected, constant time
	push	{r4, r5, r6, r7, r8, r9, r10, fp, lr}
        // r0 : swap bit in bit #0 and random data in bits #1 ... 31.
        // r1 : pFe1
        // r2 : pFe2
        // r3 : randomization mask

        sub sp,sp,#8 // reserve two stack slots.

        orr.w     r3,r3, #1  // set bit 1 in randomization mask
        eor.w     r4,r0,r3   // generate multipliaction mask with complement of swap bit in bit #0

        // clear bits #1 to 15 in the mpy swap masks.
        bic.w	r0, r0, #0xff00
        bic.w	r4, r4, #0xff00
        ror r3,r3,#13 

	bic.w	r0, r0, #0xfe
	bic.w	r4, r4, #0xfe

        bic.w   r3,r3, #0x80008000

        // determine end-of-loop counter and store to stack slot.
        add lr,r1,#28
        str lr,[sp,#0]


        // first handle most significant word
        ldr r5,[r1,#28]
        ldr r6,[r2,#28]
 
        mul r7,r0,r5
        mul r8,r0,r6
        umlal r7,r10,r4,r6
        umlal r8,r10,r4,r5

        uxth r7,r7
        uxth r8,r8

        // accumulate result
        umull r9,r10,r3,r7
        ROTATE_RANDOM_MASK
        umull fp,lr,r3,r8
        ROTATE_RANDOM_MASK

        ror r5,r5,#16
        ror r6,r6,#16

        mul r7,r0,r5
        mul r8,r0,r6
        umlal r7,r6,r4,r6
        umlal r8,r5,r4,r5

        lsl r7,r7,#16
        lsl r8,r8,#16

        // accumulate result
        umlal r9,r10,r3,r7
        ROTATE_RANDOM_MASK
        umlal fp,lr,r3,r8
        ROTATE_RANDOM_MASK

        bic.w	r5,r9, #0x80000000         
        bic.w	r6,fp, #0x80000000         
        str r5,[r2,#28]
        str r6,[r1,#28]

        lsr r9,r9,#31
        lsr fp,fp,#31

	orr r9, r9, r10, lsl #1
	orr fp, fp, lr, lsl #1

        // reduce carries
        mov r5,#19
        umull r9,r10,r9,r5
        umull fp,lr,fp,r5

.loopStart:
        str r1,[sp,#4] // save pointer r1
        ldr r5,[r1,#0]
        ldr r6,[r2,#0]
 
        mul r7,r0,r5
        mul r8,r0,r6
        umlal r7,r1,r4,r6 // use r1 as scratch reg
        umlal r8,r1,r4,r5 // use r1 as scratch reg

        uxth r7,r7
        uxth r8,r8

        // accumulate result
        umlal r9,r10,r3,r7
        ROTATE_RANDOM_MASK
        umlal fp,lr,r3,r8
        ROTATE_RANDOM_MASK

        ror r5,r5,#16
        ror r6,r6,#16

        mul r7,r0,r5
        mul r8,r0,r6
        umlal r7,r6,r4,r6
        umlal r8,r5,r4,r5

        lsl r7,r7,#16
        lsl r8,r8,#16

        // accumulate result.
        umlal r9,r10,r3,r7
        ROTATE_RANDOM_MASK
        umlal fp,lr,r3,r8
        ROTATE_RANDOM_MASK

        str r9,[r2,#0]
        ldr r1,[sp,#4] // restore pointer r1
        str fp,[r1,#0] 

        mov r9,r10
        mov fp,lr
        mov r10,#0
        mov lr,#0


        add r1,r1,#4
        add r2,r2,#4

        ldr r5,[sp,#0] // fetch end of loop marker.
        cmp r1,r5
        bne .loopStart

        // accumulate last carries
        ldr r5,[r2,#0]
        ldr r6,[r1,#0]

        add r5,r9
        add r6,fp

        str r5,[r2,#0]
        str r6,[r1,#0]

        // use dummy pops for r0,r1 for the stack slots needed.
	pop	{r0, r1, r4, r5, r6, r7, r8, r9, r10, fp, pc}
#endif

	.size	cSwapAndRandomize_asm, .-cSwapAndRandomize_asm
	.ident	"GCC: (15:6.3.1+svn253039-1build1) 6.3.1 20170620"
