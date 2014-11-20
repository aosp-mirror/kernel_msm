#!/bin/bash

export ARCH=arm
export SUBARCH=arm
export CROSS_COMPILE=~/arm-cortex_a15-linux-gnueabihf-linaro_4.7.4-2014.06/bin/arm-cortex_a15-linux-gnueabihf-

export TARGET_PREBUILT_KERNEL=$(pwd)/arch/arm/boot/zImage
