# stm32f0 submodule of [mcu-framework](https://github.com/yhsb2k/mcu-framework)

[![build](https://github.com/yhsb2k/stm32f0/workflows/build/badge.svg)](https://github.com/yhsb2k/stm32f0/actions?workflow=build)
[![license](https://img.shields.io/github/license/yhsb2k/stm32f0?color=blue)](https://github.com/yhsb2k/stm32f0/blob/master/LICENSE)

## How to build
```
git clone --recursive https://github.com/yhsb2k/stm32f0.git
cd stm32f0
make
```
**Other targets:**
* `make flash` - Upload firmware to the target
* `make erase` - Erase all memory on the target
* `make reset` - Reset the target
* Debug is available in VSCode **or** using `make debug` target to start GDB server

## Requirements
* [GNU Arm Embedded Toolchain](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads)
* [CMake](https://cmake.org/download)
* [Ninja](https://ninja-build.org)
* [Make](https://winlibs.com)
* [JLink](https://www.segger.com/downloads/jlink)
