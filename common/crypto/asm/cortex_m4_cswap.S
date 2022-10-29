#include "attributesForCortexM4.inc"

	.text
	.global	curve25519_cswap_asm
	.type	curve25519_cswap_asm, %function
	.align 4

curve25519_cswap_asm:
    .syntax unified
    push {r4-r11,r14}
    mov r8, r0 //addr state
    mov r9, r1 //
    // Subtract sp for 4 random values
    sub sp, #16
    // Generate 2 random values for mask
   	mov r0, sp
	mov r1, r0
	mov r2, #4
	mov r3, #0
	bl randombytes
	add r0, sp, #4
	mov r1, r0
	mov r2, #4
	mov r3, #0
	bl randombytes

random_number_gen:
	add r0, sp, #8
	mov r1, r0
	mov r2, #4
	mov r3, #0
	bl randombytes
	add r0, sp, #12
	mov r1, r0
	mov r2, #4
	mov r3, #0
	bl randombytes

	ldr r0, [sp, #8]
	cbnz r0, random_number_two
	b random_number_gen
random_number_two:
	ldr r1, [sp, #12]
	cbnz r1, mask
	b random_number_gen

mask:
	mov r10, r0
	mov r11, r1
	// R = R0 xor R1
	ldr r5, [sp]
	ldr r6, [sp, #4]
	eor r4, r5, r6
	// Load key value and shift left, in case of carry flip bits in R
	ldr r12, [r9, #0]
	lsls r12, r12, #1
	IT CS
	mvncs r4, r4
	// Store shifted key
	str r12, [r9, #0]

	// Add 2 512bit values onto stack
	sub sp, #128

	// i=0
	ldr r0, [r8, #32]
	umull r3, r2, r0, r10
	ldr r0, [r8, #96]
	umull r14, r12, r0, r11

	eor r0, r3, r14
	// masked and
	and r1, r0, r4	// a0b0
	and r7, r0, r5	// a0b1
	eor r1, r7		// a0b0 + a0b1
	and r7, r0, r6	// a0b2
	eor r1, r7		// a0b0 + a0b1 + a0b2
	// xor and store result
	eor r0, r3, r1
	str r0, [sp, #8]
	eor r1, r14, r1
	str r1, [sp, #72]


	// i=1
	ldr r0, [r8, #36]
	mov r3, #0
	umaal r3, r2, r0, r10
	ldr r0, [r8, #100]
	mov r14, #0
	umaal r14, r12, r0, r11

	eor r0, r3, r14
	// masked and
	and r1, r0, r4	// a0b0
	and r7, r0, r5	// a0b1
	eor r1, r7		// a0b0 + a0b1
	and r7, r0, r6	// a0b2
	eor r1, r7		// a0b0 + a0b1 + a0b2
	// xor and store result
	eor r0, r3, r1
	str r0, [sp, #12]
	eor r1, r14, r1
	str r1, [sp, #76]

	// i=2
	ldr r0, [r8, #40]
	mov r3, #0
	umaal r3, r2, r0, r10
	ldr r0, [r8, #104]
	mov r14, #0
	umaal r14, r12, r0, r11

	eor r0, r3, r14
	// masked and
	and r1, r0, r4	// a0b0
	and r7, r0, r5	// a0b1
	eor r1, r7		// a0b0 + a0b1
	and r7, r0, r6	// a0b2
	eor r1, r7		// a0b0 + a0b1 + a0b2
	// xor and store result
	eor r0, r3, r1
	str r0, [sp, #16]
	eor r1, r14, r1
	str r1, [sp, #80]

	// i=3
	ldr r0, [r8, #44]
	mov r3, #0
	umaal r3, r2, r0, r10
	ldr r0, [r8, #108]
	mov r14, #0
	umaal r14, r12, r0, r11

	eor r0, r3, r14
	// masked and
	and r1, r0, r4	// a0b0
	and r7, r0, r5	// a0b1
	eor r1, r7		// a0b0 + a0b1
	and r7, r0, r6	// a0b2
	eor r1, r7		// a0b0 + a0b1 + a0b2
	// xor and store result
	eor r0, r3, r1
	str r0, [sp, #20]
	eor r1, r14, r1
	str r1, [sp, #84]

	// i=4
	ldr r0, [r8, #48]
	mov r3, #0
	umaal r3, r2, r0, r10
	ldr r0, [r8, #112]
	mov r14, #0
	umaal r14, r12, r0, r11

	eor r0, r3, r14
	// masked and
	and r1, r0, r4	// a0b0
	and r7, r0, r5	// a0b1
	eor r1, r7		// a0b0 + a0b1
	and r7, r0, r6	// a0b2
	eor r1, r7		// a0b0 + a0b1 + a0b2
	// xor and store result
	eor r0, r3, r1
	str r0, [sp, #24]
	eor r1, r14, r1
	str r1, [sp, #88]

	// i=5
	ldr r0, [r8, #52]
	mov r3, #0
	umaal r3, r2, r0, r10
	ldr r0, [r8, #116]
	mov r14, #0
	umaal r14, r12, r0, r11

	eor r0, r3, r14
	// masked and
	and r1, r0, r4	// a0b0
	and r7, r0, r5	// a0b1
	eor r1, r7		// a0b0 + a0b1
	and r7, r0, r6	// a0b2
	eor r1, r7		// a0b0 + a0b1 + a0b2
	// xor and store result
	eor r0, r3, r1
	str r0, [sp, #28]
	eor r1, r14, r1
	str r1, [sp, #92]

	// i=6
	ldr r0, [r8, #56]
	mov r3, #0
	umaal r3, r2, r0, r10
	ldr r0, [r8, #120]
	mov r14, #0
	umaal r14, r12, r0, r11

	eor r0, r3, r14
	// masked and
	and r1, r0, r4	// a0b0
	and r7, r0, r5	// a0b1
	eor r1, r7		// a0b0 + a0b1
	and r7, r0, r6	// a0b2
	eor r1, r7		// a0b0 + a0b1 + a0b2
	// xor and store result
	eor r0, r3, r1
	str r0, [sp, #32]
	eor r1, r14, r1
	str r1, [sp, #96]

	// i=7
	ldr r0, [r8, #60]
	mov r3, #0
	umaal r3, r2, r0, r10
	ldr r0, [r8, #124]
	mov r14, #0
	umaal r14, r12, r0, r11

	eor r0, r3, r14
	// masked and
	and r1, r0, r4	// a0b0
	and r7, r0, r5	// a0b1
	eor r1, r7		// a0b0 + a0b1
	and r7, r0, r6	// a0b2
	eor r1, r7		// a0b0 + a0b1 + a0b2
	// xor and store result
	eor r0, r3, r1
	str r0, [sp, #36]
	eor r1, r14, r1
	str r1, [sp, #100]

	// store last limb
	eor r0, r2, r12
	// masked and
	and r1, r0, r4	// a0b0
	and r7, r0, r5	// a0b1
	eor r1, r7		// a0b0 + a0b1
	and r7, r0, r6	// a0b2
	eor r1, r7		// a0b0 + a0b1 + a0b2
	// xor and store result
	eor r0, r2, r1
	str r0, [sp, #40]
	eor r1, r12, r1
	str r1, [sp, #104]

	mov r0, #0
	str r0, [sp, #44]
	str r0, [sp, #48]
	str r0, [sp, #52]
	str r0, [sp, #56]
	str r0, [sp, #60]
	str r0, [sp, #64]
	str r0, [sp, #68]

	str r0, [sp, #108]
	str r0, [sp, #112]
	str r0, [sp, #116]
	str r0, [sp, #120]
	str r0, [sp, #124]
	str r0, [sp, #128]
	str r0, [sp, #132]

	add r0, r8, #32
	add r1, sp, #8
	bl fe25519_reduceTo256Bits_asm

	add r0, r8, #96
	add r1, sp, #72
	bl fe25519_reduceTo256Bits_asm

	// i=0
	ldr r0, [r8, #64]
	umull r3, r2, r0, r10
	ldr r0, [r8, #128]
	umull r14, r12, r0, r11

	eor r0, r3, r14
	// masked and
	and r1, r0, r4	// a0b0
	and r7, r0, r5	// a0b1
	eor r1, r7		// a0b0 + a0b1
	and r7, r0, r6	// a0b2
	eor r1, r7		// a0b0 + a0b1 + a0b2
	// xor and store result
	eor r0, r3, r1
	str r0, [sp, #8]
	eor r1, r14, r1
	str r1, [sp, #72]


	// i=1
	ldr r0, [r8, #68]
	mov r3, #0
	umaal r3, r2, r0, r10
	ldr r0, [r8, #132]
	mov r14, #0
	umaal r14, r12, r0, r11

	eor r0, r3, r14
	// masked and
	and r1, r0, r4	// a0b0
	and r7, r0, r5	// a0b1
	eor r1, r7		// a0b0 + a0b1
	and r7, r0, r6	// a0b2
	eor r1, r7		// a0b0 + a0b1 + a0b2
	// xor and store result
	eor r0, r3, r1
	str r0, [sp, #12]
	eor r1, r14, r1
	str r1, [sp, #76]

	// i=2
	ldr r0, [r8, #72]
	mov r3, #0
	umaal r3, r2, r0, r10
	ldr r0, [r8, #136]
	mov r14, #0
	umaal r14, r12, r0, r11

	eor r0, r3, r14
	// masked and
	and r1, r0, r4	// a0b0
	and r7, r0, r5	// a0b1
	eor r1, r7		// a0b0 + a0b1
	and r7, r0, r6	// a0b2
	eor r1, r7		// a0b0 + a0b1 + a0b2
	// xor and store result
	eor r0, r3, r1
	str r0, [sp, #16]
	eor r1, r14, r1
	str r1, [sp, #80]

	// i=3
	ldr r0, [r8, #76]
	mov r3, #0
	umaal r3, r2, r0, r10
	ldr r0, [r8, #140]
	mov r14, #0
	umaal r14, r12, r0, r11

	eor r0, r3, r14
	// masked and
	and r1, r0, r4	// a0b0
	and r7, r0, r5	// a0b1
	eor r1, r7		// a0b0 + a0b1
	and r7, r0, r6	// a0b2
	eor r1, r7		// a0b0 + a0b1 + a0b2
	// xor and store result
	eor r0, r3, r1
	str r0, [sp, #20]
	eor r1, r14, r1
	str r1, [sp, #84]

	// i=4
	ldr r0, [r8, #80]
	mov r3, #0
	umaal r3, r2, r0, r10
	ldr r0, [r8, #144]
	mov r14, #0
	umaal r14, r12, r0, r11

	eor r0, r3, r14
	// masked and
	and r1, r0, r4	// a0b0
	and r7, r0, r5	// a0b1
	eor r1, r7		// a0b0 + a0b1
	and r7, r0, r6	// a0b2
	eor r1, r7		// a0b0 + a0b1 + a0b2
	// xor and store result
	eor r0, r3, r1
	str r0, [sp, #24]
	eor r1, r14, r1
	str r1, [sp, #88]

	// i=5
	ldr r0, [r8, #84]
	mov r3, #0
	umaal r3, r2, r0, r10
	ldr r0, [r8, #148]
	mov r14, #0
	umaal r14, r12, r0, r11

	eor r0, r3, r14
	// masked and
	and r1, r0, r4	// a0b0
	and r7, r0, r5	// a0b1
	eor r1, r7		// a0b0 + a0b1
	and r7, r0, r6	// a0b2
	eor r1, r7		// a0b0 + a0b1 + a0b2
	// xor and store result
	eor r0, r3, r1
	str r0, [sp, #28]
	eor r1, r14, r1
	str r1, [sp, #92]

	// i=6
	ldr r0, [r8, #88]
	mov r3, #0
	umaal r3, r2, r0, r10
	ldr r0, [r8, #152]
	mov r14, #0
	umaal r14, r12, r0, r11

	eor r0, r3, r14
	// masked and
	and r1, r0, r4	// a0b0
	and r7, r0, r5	// a0b1
	eor r1, r7		// a0b0 + a0b1
	and r7, r0, r6	// a0b2
	eor r1, r7		// a0b0 + a0b1 + a0b2
	// xor and store result
	eor r0, r3, r1
	str r0, [sp, #32]
	eor r1, r14, r1
	str r1, [sp, #96]

	// i=7
	ldr r0, [r8, #92]
	mov r3, #0
	umaal r3, r2, r0, r10
	ldr r0, [r8, #156]
	mov r14, #0
	umaal r14, r12, r0, r11

	eor r0, r3, r14
	// masked and
	and r1, r0, r4	// a0b0
	and r7, r0, r5	// a0b1
	eor r1, r7		// a0b0 + a0b1
	and r7, r0, r6	// a0b2
	eor r1, r7		// a0b0 + a0b1 + a0b2
	// xor and store result
	eor r0, r3, r1
	str r0, [sp, #36]
	eor r1, r14, r1
	str r1, [sp, #100]

	// store last limb
	eor r0, r2, r12
	// masked and
	and r1, r0, r4	// a0b0
	and r7, r0, r5	// a0b1
	eor r1, r7		// a0b0 + a0b1
	and r7, r0, r6	// a0b2
	eor r1, r7		// a0b0 + a0b1 + a0b2
	// xor and store result
	eor r0, r2, r1
	str r0, [sp, #40]
	eor r1, r12, r1
	str r1, [sp, #104]

	mov r0, #0
	str r0, [sp, #44]
	str r0, [sp, #48]
	str r0, [sp, #52]
	str r0, [sp, #56]
	str r0, [sp, #60]
	str r0, [sp, #64]
	str r0, [sp, #68]

	str r0, [sp, #108]
	str r0, [sp, #112]
	str r0, [sp, #116]
	str r0, [sp, #120]
	str r0, [sp, #124]
	str r0, [sp, #128]
	str r0, [sp, #132]

	add r0, r8, #64
	add r1, sp, #8
	bl fe25519_reduceTo256Bits_asm

	add r0, r8, #128
	add r1, sp, #72
	bl fe25519_reduceTo256Bits_asm


	add sp, #144
	pop {r4-r11,r15}
	.size	curve25519_cswap_asm, .-curve25519_cswap_asm
