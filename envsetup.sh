#!/bin/bash

export ARCH=arm
export SUBARCH=arm
export CROSS_COMPILE=~/arm-cortex_a15-linux-gnueabihf-linaro_4.9.3-2014.11/bin/arm-eabi-
export TARGET_PREBUILT_KERNEL=$(pwd)/arch/arm/boot/zImage
