#!/bin/bash

tmp_dir=$(mktemp -d -t freebot-XXXXXXXX)
mkdir -p $tmp_dir
pushd $tmp_dir

curl https://raw.githubusercontent.com/unhuman-io/freebot/main/install-freebot.sh > install_freebot.sh
chmod +x install_freebot.sh
sudo ./install_freebot.sh

popd
rm -rf $tmp_dir