# The sca25519 library ("SCA-secure ECC in software – mission impossible?")

This repository accompanies the paper **SCA-secure ECC in software – mission impossible?** available at 
- https://eprint.iacr.org/2021/1003 and 
- https://tches.iacr.org/index.php/TCHES/issue/.

Authors:
- [Lejla Batina](https://www.cs.ru.nl/~lejla/) `lejla@cs.ru.nl`
- Łukasz Chmielewski `chmiel@fi.muni.cz` and `lukaszc@cs.ru.nl`
- Björn Haase `Bjoern.M.Haase@web.de`
- [Niels Samwel](https://nielssamwel.nl/) `niels.samwel@ru.nl`
- [Peter Schwabe](https://cryptojedi.org/peter/index.shtml) `peter@cryptojedi.org`

This repository contains three implementations of X25519 in C and assembly for the Cortex-M4 with countermeasures against side-channel and fault injection attacks. The first implementation is unprotected, the second implementation contains countermeasure required for the case of ephemeral scalar multiplication, and the third implementation contains the most countermeasures for the case of static scalar multiplication. The three implementations are located in similarly named subdirectories. The repository includes a common directory that contains code common to the three implementations and a hostside directory that contains python code to communicate with the board.

The Cortex-M4 implementations are based on [this](https://github.com/joostrijneveld/STM32-getting-started) STM32: getting started repository.

# Installation

This code assumes you have the [arm-none-eabi toolchain](https://launchpad.net/gcc-arm-embedded) installed and accessible. Confusingly, the tools available in the (discontinued) embedian project have identical names - be careful to select the correct toolchain (or consider re-installing if you experience unexpected behaviour). On most Linux systems, the correct toolchain gets installed when you install the `arm-none-eabi-gcc` (or `gcc-arm-none-eabi`) package. Besides a compiler and assembler, you may also want to install `arm-none-eabi-gdb`. On Linux Mint, be sure to explicitly install `libnewlib-arm-none-eabi` as well (to fix an error relating to `stdint.h`).

This project relies on the [libopencm3](https://github.com/libopencm3/libopencm3/) firmware. This is included as a submodule and we also included it directly to the folder libopencm3 in the main directory. Compile it (e.g. by calling `make lib` in one of the platform-specific directories) before attempting to compile any of the other targets.

The binary can be compiled by calling `make` in each respective subdirectory (unprotected, ephemeral, static). The binary can then be flashed onto the boards using [stlink](https://github.com/texane/stlink), as follows: `st-flash write main.bin 0x8000000`. Depending on your operating system, stlink may be available in your package manager -- otherwise refer to their Github page for instructions on how to [compile it from source](https://github.com/texane/stlink/blob/master/doc/compiling.md) (in that case, be careful to use libusb-1.0.0-dev, not libusb-0.1).

The host-side Python code requires the [pyserial](https://github.com/pyserial/pyserial) module. Your package repository might offer `python-serial` or `python-pyserial` directly (as of writing, this is the case for Ubuntu, Debian and Arch). Alternatively, this can be easily installed from PyPA by calling `pip install pyserial` (or `pip3`, depending on your system). If you do not have `pip` installed yet, you can typically find it as `python3-pip` using your package manager. Use the `host_unidirectional.py` script to receive data from the board.

## Requirements Summary
- `arm-none-eabi-gcc` with version 9.2.1 20191025 (with -O2 optimization flag); all related work for performance evaluation we also compiled using this compiler and the -02 flag. 
- `python3` with `pyserial`
- For Cortex-M4F:
    - Board `stm32f4discovery` (we use floating-point registers)
    - `stlink`

### `libopencm3`
For our evaluation we have used `libopencm3` (https://github.com/libopencm3/libopencm3/) with the commit id: `7daa6f15bf8db77b3225df01e427777b202b4e4e` (from February 5th, 17:22:55, 2019). We have also included it in the libopencm3 directory in our repository. 

## Hooking up an STM32 discovery board

Connect the board to your machine using the mini-USB port. This provides it with power, and allows you to flash binaries onto the board. It should show up in `lsusb` as `STMicroelectronics ST-LINK/V2`.

If you are using a UART-USB connector that has a PL2303 chip on board (which appears to be the most common), the driver should be loaded in your kernel by default. If it is not, it is typically called `pl2303`. On macOS, you will still need to [install it](http://www.prolific.com.tw/US/ShowProduct.aspx?p_id=229&pcid=41) (and reboot). When you plug in the device, it should show up as `Prolific Technology, Inc. PL2303 Serial Port` when you type `lsusb`.

Using dupont / jumper cables, connect the `TX`/`TXD` pin of the USB connector to the `PA3` pin on the board, and connect `RX`/`RXD` to `PA2`. Depending on your setup, you may also want to connect the `GND` pins.

## Troubleshooting

At some point the boards might behave differently than one would expect, to a point where simply power-cycling the board does not help. In these cases, it is useful to be aware of a few trouble-shooting steps.

## Problems related to the tools

If you're using Ubuntu, a common issue when using stlink is an error saying you are missing `libstlink-shared.so.1`. In this case, try running [`ldconfig`](https://github.com/texane/stlink/blob/master/doc/compiling.md#fixing-cannot-open-shared-object-file).

If you are running into permission errors when trying to access the serial devices as a non-root user, you could consider adding your current user to the `dialout` (Debian, Ubuntu) or `uucp` (Arch) group, using something along the lines of `sudo usermod -a -G [group] [username]`.

If you are getting Python errors when running the host-side scripts, make sure you are using Python 3.

### Problems related to the board

First, check if all the cables are attached properly. For the boards supported in this repository, connect TX to `PA3`, RX to `PA2` and GND to `GND`. Power is typically supplied using the mini-USB connector that is also used to flash code onto the board.

If the code in this repository does not appear to work correctly after flashing it on to the board, try pressing the `RST` button (optionally followed by re-flashing).

If you cannot flash new code onto the board, but are instead confronted with `WARN src/stlink-common.c: unknown chip id!`, try shorting the `BOOT0` and `VDD` pins and pressing `RST`. This selects the DFU bootloader. After that, optionally use `st-flash erase` before re-flashing the board.

If you cannot flash the code onto the board, and instead get `Error: Data length doesn't have a 32 bit alignment: +2 byte.`, make sure you are using a version of stlink for which [this issue](https://github.com/texane/stlink/issues/390) has been resolved. This affected L0 and L1 boards.

# Structure of this repository
- `STM32F407-ephemeral` contains our implementation for the ephemeral X25519; this implementation contains some side-channel protections.
- `STM32F407-static` contains our implementation for the static X25519; this implementation contains multiple side-channel protections.
- `STM32F407-unprotected` contains our implementation for the unprotected X25519; this implementation contains no side-channel protections besides being constant-time.
- `common` contains common files for the implementations. Currently these files are copied to the implementations but symbolic links can be used instead. 
- `hostside` contains simple python code to communicate with the device. We use it to read the test results (for the performance evaluation) from the board. 

## Relevant Flags

The following flags are relevant and useful for performance evaluation: 
- `ITOH_COUNTERMEASURE` and `ITOH_COUNTERMEASURE64` (in file `scalarmult_25519.c`) specify whether the address randomization is turned on in the static multiplication; both are turned on by default; 
- `UPDATABLE_STATIC_SCALAR` (in file `scalarmult_25519.c`) specifies whether the static scalar is being updated per each scalar multiplication call and `SCALAR_RANDOMIZATION` (also in `scalarmult_25519.c`) specifies whether the scalar is updated just before the scalar multiplication (turning these countermeasures of is important for template attacks); both flags are turned on by default; 
- `COUNT_CYCLES_EXTRA_SM` (in file `crypto_scalarmult.h`) is only useful when measuring clock cycles that the extra 64-bit scalar multiplication takes in the static implementation; otherwise, the flag should be turned off because it affects the efficiency of the scalar multiplication; it is turned off by default;
- `MULTIPLICATIVE_CSWAP` (in file `crypto_scalarmult.h`) is a flag that can be used for two different implementations of the `cswaperr` procedure (for both, the ephemeral and static implementations). All the results in the paper are presented when `MULTIPLICATIVE_CSWAP` is enabled, since this implementation occurred to be safer and more efficient. 

# Traces

As mentioned in the paper we run our side-channel experiments using a modified Cortex-M4 (several capacitors are removed) on an STM32F407IGT6 board clocked at 168MHz. 
The resulting comparison of traces corresponding to all three implementations is present below: 
![alt text](https://github.com/sca-secure-library-sca25519/sca25519/blob/main/figs/EphemeralSM_mark2.png?raw=true "Ephemeral Implementation")
![alt text](https://github.com/sca-secure-library-sca25519/sca25519/blob/main/figs/StaticSM_mark2.png?raw=true "Static Implementation")
![alt text](https://github.com/sca-secure-library-sca25519/sca25519/blob/main/figs/UnprotectedSM_mark2.png?raw=true "Unprotected Implementation")


The scalar multiplications are marked in red. 

To help reproduce the side-channel evaluation, for each of the implementations we upload 100 traces under the following link: https://www.dropbox.com/scl/fo/3zjd5m4oc77c5qgob7kxf/h?dl=0&rlkey=59774fmckgv5k8z87dbk7gpqk.

The first byte of the attached data to each trace indicates the TVLA set (0 - fixed, 1 - random input and fixed key, and 2 fixed input and random key). The rest of the data corresponds to the input point and the scalar. 

## Format

Each trace set is in the TRS format that is described under the following links:
- https://github.com/Riscure/python-trsfile
- https://github.com/Riscure/java-trsfile
- https://github.com/Riscure/Jlsca 

The python `trsfile` package can be used to read the traces and it can be installed using pip: https://pypi.org/project/trsfile/. 

# License
All our code is covered by CC0. `libopencm3` is licensed under GPL version 3, see https://github.com/libopencm3/libopencm3.

