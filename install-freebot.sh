#!/bin/bash

installs=(
    https://github.com/unhuman-io/usb_rt_driver/releases/download/0.6.0/usb_rt_driver-0.6.0-Linux.deb 
    https://github.com/unhuman-io/realtime-tmp/releases/download/0.4.0/realtime-tmp-0.4.0-Linux.deb
)

sudo apt install -y dkms libudev1 dfu-util wget
mkdir -p freebot-tmp
cd freebot-tmp

for install in ${installs[@]}; do
    wget $install
    sudo dpkg -i $(basename $install)
done

cd ..
rm -r freebot-tmp
