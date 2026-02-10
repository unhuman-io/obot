#!/bin/bash -e

sudo apt install -y libpopt-dev
wget https://www.peak-system.com/quick/PCAN-Linux-Driver -O pcan-linux-driver.tar.gz
tar xf pcan-linux-driver.tar.gz
pushd peak-linux-driver-*
make -j netdev #CC=gcc-12
sudo make install_with_dkms #CC=gcc-12
sudo sed -i '/fast_fwd/s/^# //g' /etc/modprobe.d/pcan.conf
sudo sed -i 's/# options pcan fdirqcl=.*/options pcan fdirqcl=1/' /etc/modprobe.d/pcan.conf
sudo sed -i 's/# options pcan fdirqtl=.*/options pcan fdirqtl=1/' /etc/modprobe.d/pcan.conf
echo "options pcan assign=devid" | sudo tee -a /etc/modprobe.d/pcan.conf

cat << EOF | sudo tee -a /etc/systemd/network/20-pcan-canfd.network
[Match]
Type=can

[CAN]
BitRate=1000000
DataBitRate=8000000
FDMode=yes
RestartSec=100ms

EOF

sudo systemctl enable systemd-networkd
sudo systemctl restart systemd-networkd
