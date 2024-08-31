# Hey Emacs, this is a -*- makefile -*-
#----------------------------------------------------------------------------
#
# Makefile for ChipWhisperer SimpleSerial-AES Program
#
#----------------------------------------------------------------------------
# On command line:
#
# make all = Make software.
#
# make clean = Clean out built project files.
#
# make coff = Convert ELF to AVR COFF.
#
# make extcoff = Convert ELF to AVR Extended COFF.
#
# make program = Download the hex file to the device, using avrdude.
#                Please customize the avrdude settings below first!
#
# make debug = Start either simulavr or avarice as specified for debugging,
#              with avr-gdb or avr-insight as the front end for debugging.
#
# make filename.s = Just compile filename.c into the assembler code only.
#
# make filename.i = Create a preprocessed source file for use in submitting
#                   bug reports to the GCC project.
#
# To rebuild project do "make clean" then "make all".
#----------------------------------------------------------------------------

CRYPTO_TARGET = NONE

# Target file name (without extension).
# This is the base name of the compiled .hex file.
TARGET = simpleserial_sca25519

# List C source files here.
# Header files (.h) are automatically pulled in.
SRC+=$(wildcard *.c)
SRC+=$(wildcard crypto/numerics/*.c)
SRC+=$(wildcard crypto/scalarmult/*.c)
SRC+=$(wildcard crypto/support/*.c)
#EXTRAINCDIRS += crypto
#EXTRAINCDIRS += crypto/numerics
#EXTRAINCDIRS += crypto/include
#EXTRAINCDIRS += libopencm3/include
ASRC += $(wildcard crypto/asm/*.S)
#ASRC += $(wildcard crypto/asm/*.inc)

OPT = 2
CDEFS += -DSTM32F4 -DCORTEX_M4 -mthumb -mcpu=cortex-m4 -mfpu=fpv4-sp-d16
#-DWITH_PERFORMANCE_BENCHMARKING 
EXTRALIBDIRS += libopencm3/lib
FPUUSE = 1


# -----------------------------------------------------------------------------
CFLAGS += $(EXTRA_OPTS)

${info Building for platform ${PLATFORM} with CRYPTO_TARGET=$(CRYPTO_TARGET)}

#Add simpleserial project to build
include ../simpleserial/Makefile.simpleserial

FIRMWAREPATH = ../.
include $(FIRMWAREPATH)/Makefile.inc

