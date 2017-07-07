#!/bin/bash
export ARCH=arm
export CROSS_COMPILE=arm-linux-gnueabi-
#make distclean;

make imx_v7_defconfig


make -j4 zImage LOADADDR=0x10008000

make imx6d-ginhmi.dtb

make modules


# Copy all modules into a debug folder called dbg_lib.
#The Yocto build automatically copies the modules into the rootfs

make INSTALL_MOD_PATH=$(pwd)/dbg_lib modules_install

tar -cf kernel-modules.tar -C dbg_lib .

rm -rf dbg_lib