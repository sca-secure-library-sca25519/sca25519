# Capture a trace from the static implementation

This is a Chipwhisperer acquisition code for the static implementation. 

All the files in the root directory `main.h`, `main.h`, `main.c`, `test.h`, `test.c`, and `Makefile` are adjusted for Chipwhisperer. All the files, but two, in the `crypto` directory correspond to the release 1.0. The first difference is `crypto/support/randombytes.c` - this if PRNG that can also be used on F3 targets. If you have an F4 target then overwrite `randombytes.c` with `./crypto/support/randombytes_CW308_STM32F4.c` (that is the original TRNG like in the repository). The second difference also relates to randomness: we needed to comment out the first line in `crypto/scalarmult/scalarmult_25519.c` because TRNG is not supported.

## How to run?

Run `sca25519.ipynb` in jupiter notebook. 

As a result you should get a trace looking more or less as follows: 
![alt text](./fig/static_all_cw.png?raw=true "Static Implementation")

## Disabling countermeasures

If you would like to evaluate the attack against profiled attacks then you need to remove scalar randomizations to be able to label iterations of the scalar multiplication loop. The following defines need to be commented out in `scalarmult_25519.c` to achieve that: 
```
//#define UPDATABLE_STATIC_SCALAR
//#define SCALAR_RANDOMIZATION
```

If you would like additionally to remove the countermeasure against template attacks then would need to comment out two defines in `scalarmult_25519.c` to achieve that: 
```
//#define ITOH_COUNTERMEASURE
//#define ITOH_COUNTERMEASURE64
```

If all four defines are commented out then you should get a trace looking more or less as follows: 
![alt text](./fig/static_not_all_cw.png?raw=true "Static Implementation With Disabled Countermeasures")

## Providing masked scalar

If you would like to provide a masked scalar to the implementation then you would need to implement such a code in `main.c` (at the moment it is not implemented). However, the library provides a function to set the key values accordingly: `set_static_key_curve25519` in `scalarmult_25519.c`. 

Since the key consist of multiple components it is not trivial to generate a proper key, we provide a [sage](https://www.sagemath.org/) script to do that - see `./sage/genBlindingPoints.sage`.

## Comment

The current code just captures a single trace for a hardcoded test vector. If you want to randomize the input or the key - this code would need to be added to the code (currently the data provided by the command is not used). 

Additionally, if you would like to reproduce the results from the Ches paper a TRNG should be used. 
