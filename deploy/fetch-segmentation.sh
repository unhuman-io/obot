#!/bin/sh
# Runs on the robot as root: fetches the segmentation network (checked against its sum) to
# /var/lib/obot/models and makes it ready for this robot's GPU, which takes minutes the
# first time. Safe to run again. Needs obot-segd installed and the internet.
#
# The network is the FCN-ResNet18 trained on SUN RGB-D from the jetson-inference project
# (github.com/dusty-nv/jetson-inference, MIT): 21 classes of things in rooms.
set -eu
name=FCN-ResNet18-SUN-RGBD-512x400
sha256=4dd90c3145164907b00ed436d6179935d65f75505cc0386d743fba3c23e6a0cd
dir=/var/lib/obot/models
model="$dir/$name/fcn_resnet18.onnx"
if [ ! -s "$model" ]; then
    mkdir -p "$dir"
    archive="$(mktemp)"
    curl -fsSL -o "$archive" "https://github.com/dusty-nv/jetson-inference/releases/download/model-mirror-190618/$name.tar.gz"
    echo "$sha256  $archive" | sha256sum -c - >/dev/null
    tar -C "$dir" --no-same-owner -xzf "$archive"
    rm -f "$archive"
fi
[ -s "$model.engine" ] || obot-segd --build "$model"
chmod -R a+rX "$dir"
echo "$model"
