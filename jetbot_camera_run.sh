#!/bin/bash

gst-launch-1.0 nvarguscamerasrc ! nvvidconv ! 'video/x-raw(memory:NVMM), width=(int)1024, height=(int)720, format=(string)I420, framerate=(fraction)30/1' ! omxh264enc ! 'video/x-h264, stream-format=(string)byte-stream' ! h264parse ! rtph264pay mtu=1400 ! udpsink host=jetson.local port=5000 sync=false async=false &
gst-launch-1.0 v4l2src device="/dev/video1" ! 'video/x-raw,framerate=30/1' ! omxh264enc ! 'video/x-h264, stream-format=(string)byte-stream' ! h264parse ! rtph264pay mtu=1400 ! udpsink host=jetson.local port=5001 sync=false async=false &
gst-launch-1.0 v4l2src device="/dev/video2" ! 'video/x-raw,framerate=30/1' ! omxh264enc ! 'video/x-h264, stream-format=(string)byte-stream' ! h264parse ! rtph264pay mtu=1400 ! udpsink host=jetson.local port=5002 sync=false async=false &
