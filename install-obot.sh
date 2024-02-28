#!/bin/bash

set -eo pipefail

tmp_dir=$(mktemp -d -t obot-XXXXXXXX)
arch=${arch:-$(uname -m)}
usb_rt_version=0.7.4

echo "install branch: ${branch:=develop}"
system_installs=()
if [ ! -z $1 ] && [ $1 == "--no-driver" ]; then
    echo "not installing usb rt driver"
else
    driver_deps=dkms
    if [ ! "$(modinfo usb_rt 2> /dev/null)" ] || [ $(modinfo -F version usb_rt) != "${usb_rt_version}" ]; then
        system_installs+=(https://github.com/unhuman-io/usb_rt_driver/releases/download/${usb_rt_version}/usb_rt_driver-${usb_rt_version}-${arch}.deb)
    fi
fi

installs=(
    https://github.com/unhuman-io/motor-realtime/releases/download/${branch}/motor-realtime-${arch}.deb
)

apt_deps=(libudev1 dfu-util wget $driver_deps)
apt_installs=()
for dep in ${apt_deps[@]}; do
    if ! dpkg -l $dep > /dev/null; then
        apt_installs+=($dep)
    fi
done
if [ ${#apt_installs[@]} -ne 0 ]; then
    echo "apt installing " ${apt_installs[@]}
    sudo apt install -y ${apt_installs[@]}
fi

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
    printf "\nInstalling motor_gui to /usr/bin\n"
    gui_branch=main
    if [ $branch == "0.14" ]; then
        gui_branch=v0.4  
    fi
    wget https://github.com/unhuman-io/obot-demo-gui/releases/download/$gui_branch/dist.zip
    unzip dist.zip > /dev/null
    chmod +x dist/motor_gui/motor_gui
    sudo rm -rf /usr/bin/motor_gui_lib
    sudo mv dist/motor_gui /usr/bin/motor_gui_lib
    sudo ln -sf /usr/bin/motor_gui_lib/motor_gui /usr/bin/motor_gui 
fi


popd
rm -rf $tmp_dir
