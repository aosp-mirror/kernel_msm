#!/bin/bash

export ARCH=arm
export SUBARCH=arm
export CROSS_COMPILE=~/arm-cortex_a15-linux-gnueabihf-linaro_4.9.2-2014.10/bin/arm-eabi-

export TARGET_PREBUILT_KERNEL=$(pwd)/arch/arm/boot/zImage
