#!/bin/bash
sudo apt install -y cmake git dkms libudev-dev dfu-util

# seems that this needs to be done for some sort of bug in the image
# https://forum.loverpi.com/discussion/555/how-to-fix-dkms-error-bin-sh-1-scripts-basic-fixdep-exec-format-error
if [ -d /usr/src/linux-headers-$(uname -r)/kernel-4.9 ]; then
    cd /usr/src/linux-headers-$(uname -r)/kernel-4.9
    scripts/basic/fixdep
    if [ $? -ne 1 ]; then
        sudo make scripts
    fi
fi

mkdir ~/unhuman_tmp
cd ~/unhuman_tmp
git clone https://github.com/unhuman-io/usb_rt_driver
cd usb_rt_driver
mkdir build
cd build
cmake ..
make package
sudo dpkg -i packages/*.deb

cd ../..
git clone https://github.com/unhuman-io/realtime-tmp
cd realtime-tmp
git checkout develop
mkdir buildr
cd buildr
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j4 package
sudo dpkg -i packages/*.deb