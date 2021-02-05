#!/bin/bash

gst-launch-1.0 udpsrc port=5000 ! application/x-rtp,encoding-name=H264,payload=96 ! rtph264depay ! queue ! h264parse ! nvv4l2decoder ! nvoverlaysink overlay-w=1920 overlay-h=1080 &
gst-launch-1.0 udpsrc port=5001 ! application/x-rtp,encoding-name=H264,payload=96 ! rtph264depay ! queue ! h264parse ! nvv4l2decoder ! nvoverlaysink overlay-w=1920 overlay-h=1080 overlay-x=1920 overlay=2 &
gst-launch-1.0 udpsrc port=5002 ! application/x-rtp,encoding-name=H264,payload=96 ! rtph264depay ! queue ! h264parse ! nvv4l2decoder ! nvoverlaysink overlay-w=1920 overlay-h=1080 overlay-y=1080 overlay=0 &


# for laptop
#gst-launch-1.0 udpsrc port=5001 ! application/x-rtp,encoding-name=H264,payload=96 ! rtph264depay ! decodebin ! videoconvert ! autovideosink