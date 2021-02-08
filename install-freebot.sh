#!/bin/bash

tmp_dir=$(mktemp -d -t freebot-XXXXXXXX)
arch=$(uname -m)
usb_rt_version=0.6.2

system_installs=()
modinfo usb_rt 2> /dev/null
if [ $? -ne 0 ] || [ $(modinfo -F version usb_rt) != "${usb_rt_version}" ]; then
    system_installs+=(https://github.com/unhuman-io/usb_rt_driver/releases/download/${usb_rt_version}/usb_rt_driver-${usb_rt_version}-${arch}.deb)
fi

installs=(
    https://github.com/unhuman-io/realtime-tmp/releases/download/develop/realtime-tmp-${arch}.deb
)

sudo apt install -y dkms libudev1 dfu-util wget

mkdir -p $tmp_dir
pushd $tmp_dir

curl https://raw.githubusercontent.com/unhuman-io/freebot/main/update-freebot.sh > update-freebot.sh
chmod +x update-freebot.sh
sudo cp update-freebot.sh /usr/bin/update-freebot

for install in ${system_installs[@]}; do
    wget $install
    sudo dpkg -i $(basename $install)
done

for install in ${installs[@]}; do
    wget $install
    sudo dpkg -i $(basename $install)
done


popd
rm -rf $tmp_dir
