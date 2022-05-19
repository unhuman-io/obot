#!/bin/bash

tmp_dir=$(mktemp -d -t obot-XXXXXXXX)
mkdir -p $tmp_dir
pushd $tmp_dir

curl https://raw.githubusercontent.com/unhuman-io/obot/main/install-obot.sh > install_obot.sh
chmod +x install_obot.sh
./install_obot.sh $@

popd
rm -rf $tmp_dir