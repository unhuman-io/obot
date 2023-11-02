#!/bin/bash

set -eo pipefail

tmp_dir=$(mktemp -d -t obot-XXXXXXXX)
arch=${arch:-$(uname -m)}
usb_rt_version=0.7.3

echo "install branch: ${branch:=develop}"
system_installs=()
if [ ! -z $1 ] && [ $1 == "--no-driver" ]; then
    echo "not installing usb rt driver"
else
    if [ ! "$(modinfo usb_rt 2> /dev/null)" ] || [ $(modinfo -F version usb_rt) != "${usb_rt_version}" ]; then
        system_installs+=(https://github.com/unhuman-io/usb_rt_driver/releases/download/${usb_rt_version}/usb_rt_driver-${usb_rt_version}-${arch}.deb)
    fi
    # helpful to have st dfu rules
    sudo sh -c "echo 'SUBSYSTEMS==\"usb\", ATTR{idVendor}==\"0483\", ATTR{idProduct}==\"df11\", MODE=\"0666\"' > /etc/udev/rules.d/99-st-dfu.rules"
fi

installs=(
    https://github.com/unhuman-io/motor-realtime/releases/download/${branch}/motor-realtime-${arch}.deb
)

sudo apt install -y dkms libudev1 dfu-util wget

mkdir -p $tmp_dir
pushd $tmp_dir

curl https://raw.githubusercontent.com/unhuman-io/obot/main/update-obot.sh > update-obot.sh
chmod +x update-obot.sh
sudo cp update-obot.sh /usr/bin/update-obot

for install in ${system_installs[@]}; do
    wget $install
    sudo dpkg -i $(basename $install)
done

for install in ${installs[@]}; do
    wget $install
    sudo dpkg -i $(basename $install)
done

# todo make package for motor_gui
if [ $arch == "x86_64" ]; then 
    wget https://github.com/unhuman-io/obot-demo-gui/releases/download/main/dist.zip
    unzip dist.zip
    chmod +x dist/motor_gui/motor_gui
    sudo rm -rf /usr/bin/motor_gui_lib
    sudo mv dist/motor_gui /usr/bin/motor_gui_lib
    sudo ln -sf /usr/bin/motor_gui_lib/motor_gui /usr/bin/motor_gui 
fi


popd
rm -rf $tmp_dir
