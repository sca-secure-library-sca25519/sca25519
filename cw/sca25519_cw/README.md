# Capture a trace from the unprotected implementation

This is a Chipwhisperer acquisition code for the unprotected implementation. 

All the files in the root directory `main.h`, `main.h`, `main.c`, `test.h`, `test.c`, and `Makefile` are adjusted for Chipwhisperer. All the files, but one, in the `crypto` directory correspond to the release 1.0. The only difference is `crypto/support/randombytes.c` - this if PRNG that can also be used on F3 targets. If you have an F4 target then overwrite `randombytes.c` with `./crypto/support/randombytes_CW308_STM32F4.c` (that is the original TRNG like in the repository). 

## How to run?

Run `sca25519.ipynb` in jupiter notebook. 

As a result you should get a trace looking more or less as follows: 
![alt text](./fig/unprotected_cw.png?raw=true "Unprotected Implementation")

## Comment

The current code just captures a single trace for a hardcoded test vector. If you want to randomize the input or the key - this code would need to be added to the code (currently the data provided by the command is not used). 

Additionally, if you would like to reproduce the results from the Ches paper a TRNG should be used. 
