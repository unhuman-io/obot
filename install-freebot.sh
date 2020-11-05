#!/bin/bash

tmp_dir=$(mktemp -d -t freebot-XXXXXXXX)

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

mkdir -p $tmp_dir
pushd $tmp_dir

curl https://raw.githubusercontent.com/unhuman-io/freebot/main/update-freebot.sh > update-freebot.sh
chmod +x update-freebot.sh
sudo cp update-freebot.sh /usr/local/bin/update-freebot

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

popd
rm -rf $tmp_dir
