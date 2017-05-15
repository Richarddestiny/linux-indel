#!/bin/bash
export ARCH=arm
export CROSS_COMPILE=arm-linux-gnueabi-
#make distclean;

make imx_v7_defconfig


make -j4 zImage LOADADDR=0x10008000

make imx6d-ginhmi.dtb

make modules
