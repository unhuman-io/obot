#!/bin/bash

system_installs=(
    https://github.com/unhuman-io/usb_rt_driver/releases/download/0.6.0/usb_rt_driver-0.6.0-Linux.deb 
)

installs=(
    https://github.com/unhuman-io/realtime-tmp/releases/download/0.4.0/realtime-tmp-0.4.0-Linux.deb
)

local_installs=(
     https://github.com/unhuman-io/realtime-tmp/releases/download/0.4.0/artifacts-0.4.0.tgz
)

sudo apt install -y dkms libudev1 dfu-util wget
mkdir -p freebot-tmp
cd freebot-tmp

for install in ${system_installs[@]}; do
    wget $install
    sudo dpkg -i $(basename $install)
done

if [ $2 == "--local" ]; then
    for install in ${installs[@]}; do
        wget $install
        sudo dpkg -i $(basename $install)
    done
else 
    for install in ${local_installs[@]}; do
        wget $install
        tar xzf $(basename $install) -C ../
    done
fi


cd ..
rm -r freebot-tmp
