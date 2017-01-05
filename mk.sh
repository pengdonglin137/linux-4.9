#!/bin/bash


make ARCH=arm uImage LOADADDR=0x40008000 -j4

make dtbs

cp arch/arm/boot/uImage /tftpboot
cp arch/arm/boot/dts/exynos4412-tiny4412.dtb  /tftpboot/dtb

dtc -I dtb -O dts -o tiny4412.dts arch/arm/boot/dts/exynos4412-tiny4412.dtb
