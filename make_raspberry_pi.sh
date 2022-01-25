#!/bin/bash

tmp_dir=$(mktemp -d -t obot-pi-XXXXXXXX)
pushd $tmp_dir
wget https://downloads.raspberrypi.org/raspios_lite_armhf/images/raspios_lite_armhf-2021-01-12/2021-01-11-raspios-buster-armhf-lite.zip
unzip 2021-01-11-raspios-buster-armhf-lite.zip

#https://wiki.debian.org/RaspberryPi/qemu-user-static
dd if=/dev/zero bs=1M count=1024 >> 2021-01-11-raspios-buster-armhf-lite.img

loop=$(sudo losetup -f -P --show 2021-01-11-raspios-buster-armhf-lite.img)
echo $loop

sudo parted -s -a opt $loop "resizepart 2 100%"
sudo e2fsck -f ${loop}p2
sudo resize2fs ${loop}p2
sudo mount ${loop}p2 -o rw /mnt

# install obot
cd /mnt
sudo cp /usr/bin/qemu-arm-static usr/bin
sudo cp -L /etc/resolv.conf etc/
sudo systemd-nspawn -D . -M tmpjetson --resolv-conf off --pipe /bin/bash << EOF
apt update
apt install -y curl
curl https://raw.githubusercontent.com/unhuman-io/obot/main/install-obot.sh > install-obot.sh
chmod +x install-obot.sh
arch=armv6l ./install-obot.sh
rm install-obot.sh
EOF
sudo rm usr/bin/qemu-arm-static
sudo rm etc/resolv.conf
pushd usr/local/src
sudo sh -c 'curl https://storage.googleapis.com/git-repo-downloads/repo > repo'
sudo chmod a+rx repo
sudo git config --global user.name "Root"
sudo git config --global user.email "root@root.com"
sudo git config --global color.ui true
sudo ./repo init -b main -u https://github.com/unhuman-io/obot
sudo ./repo sync
popd

sudo losetup -d $loop
sudo umount /mnt

popd
