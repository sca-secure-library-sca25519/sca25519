# The sca25519 library ChipWhisper code

In this folder we have three folders: 
- [sca25519_cw](sca25519_cw) - the unprotected implementation folder; 
- [sca25519_cw_eph](sca25519_cw_eph) - the ephemeral implementation folder;
- [sca25519_cw_st](sca25519_cw_st) - the ephemeral implementation folder. 

Each folder contains both the source as well as the jupyter notebook for communication and acquisition of a single trace. 
The captured traces are full and trigerring is done at the beginning of the execution of scalar multiplication. 
Currently test vectors are used to perform the operation. 

The code in this folders correspond to the release 1.0 of sca25519. For the sake of simplicity symbolic links are not used (i.e., files are simply copied here from the release 1.0 code). In each folder README.me we mention which files are potentially different/modified, but this modifications are kept to minimum. 

It was tested on: 
- CW308_STM32F4 with true random generation like the original implementation.
- CWLITEARM with PRNG.

To obtain more samples at better quality use different model, for example, Husky. Note that this device was not yet tested. 

## Instructions

Let us assume that we want to capture sca25519_cw. The instructions for other folders are the same. 

Follow the following steps: 
1. Download and install [Chipwshiperer](https://github.com/newaetech/chipwhisperer/tree/master). Do not forget to install also [jupyter notebook](https://jupyter.org/).
2. Let's consider chipwshiperer as the base folder. Copy the whole folder sca25519_cw to `./hardware/victims/firmware/`
3. Modifiy the `../Makefile.inc` in the firmware folder to create subfolders in build directory for sources in crypto, because for some reason it would not create automatically otherwise.. :
```
$(OBJDIR)/%.o : %.c
	@$(ECHO_BLANK)
	@echo $(MSG_COMPILING) $<
	@mkdir -p $(@D)
	$(CC) -c $(ALL_CFLAGS) $< -o $@

...

$(OBJDIR)/%.o : %.cpp
	@$(ECHO_BLANK)
	@echo $(MSG_COMPILING_CPP) $<
	@mkdir -p $(@D)
	$(CC) -c $(ALL_CPPFLAGS) $< -o $@

...

$(OBJDIR)/%.o : %.S
	@$(ECHO_BLANK)
	@echo $(MSG_ASSEMBLING) $<
	@mkdir -p $(@D)
	$(CC) -c $(ALL_ASFLAGS) $< -o $@
```
4. In particular remember about setting randomness in the right way
- For CW308_STM32F4 use the original file from the repository. For simplicity we include it in the folder with name `hardware/victims/firmware/sca25519_cw/crypto/support/randombytes_CW308_STM32F4.c`. Simply rename this file and overwrite `randombytes.c`. 
- For CWLITEARM (with F3 target) use the provided file: `hardware/victims/firmware/sca25519_cw/crypto/support/randombytes.c`. 
5. Follow potential extra instructions [README.md](./sca25519_cw/README.md) in the desired folder. 
6. Run `sca25519.ipynb` in jupter notebook. 