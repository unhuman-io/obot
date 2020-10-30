#!/bin/bash

tmp_dir=$(mktemp -d -t freebot-XXXXXXXX)
mkdir -p $tmp_dir
pushd $tmp_dir

curl https://raw.githubusercontent.com/unhuman-io/freebot/master/install-freebot.sh > install_freebot.sh
chmod +x install_freebot.sh
sudo ./install_freebot.sh

popd
rm -r $tmp_dir