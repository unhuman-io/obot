#!/bin/bash -e

sudo apt install -y libpopt-dev
wget https://www.peak-system.com/quick/PCAN-Linux-Driver -O pcan-linux-driver.tar.gz
tar xf pcan-linux-driver.tar.gz
pushd peak-linux-driver-*
make -j netdev CC=gcc-12
sudo make install_with_dkms CC=gcc-12
sudo sed -i '/fast_fwd/s/^# //g' /etc/modprobe.d/pcan.conf
