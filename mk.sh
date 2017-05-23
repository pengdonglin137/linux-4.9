#!/bin/bash

make tq2440_dt_defconfig
make uImage -j4
make dtbs

#cp arch/arm/boot/uImage /tftpboot
#cp arch/arm/boot/dts/s3c2440-tq2440-dt.dtb  /tftpboot/dtb

dtc -I dtb -O dts -o tq2440.dts arch/arm/boot/dts/s3c2440-tq2440-dt.dtb
