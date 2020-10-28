#!/bin/bash

system_installs=(
    https://github.com/unhuman-io/usb_rt_driver/releases/download/0.6.1/usb_rt_driver-0.6.1-Linux.deb 
)

installs=(
    https://github.com/unhuman-io/realtime-tmp/releases/download/develop/realtime-tmp.deb
)

local_installs=(
     https://github.com/unhuman-io/realtime-tmp/releases/download/develop/artifacts.tgz
)

sudo apt install -y dkms libudev1 dfu-util wget
mkdir -p freebot-tmp
cd freebot-tmp

for install in ${system_installs[@]}; do
    wget $install
    sudo dpkg -i $(basename $install)
done

if [ $1 == "--local" ]; then
    for install in ${local_installs[@]}; do
        wget $install
        tar xzf $(basename $install) -C ../
    done
else 
    for install in ${installs[@]}; do
        wget $install
        sudo dpkg -i $(basename $install)
    done
fi

cd ..
rm -r freebot-tmp
