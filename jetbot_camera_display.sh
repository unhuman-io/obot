#!/bin/bash

gst-launch-1.0 udpsrc port=5000 ! application/x-rtp,encoding-name=H264,payload=96 ! rtph264depay ! queue ! h264parse ! nvv4l2decoder ! nvdrmvideosink plane_id=1 &
gst-launch-1.0 udpsrc port=5001 ! application/x-rtp,encoding-name=H264,payload=96 ! rtph264depay ! queue ! h264parse ! nvv4l2decoder ! nvdrmvideosink offset-x=1920 plane_id=1 &
gst-launch-1.0 udpsrc port=5002 ! application/x-rtp,encoding-name=H264,payload=96 ! rtph264depay ! queue ! h264parse ! nvv4l2decoder ! nvdrmvideosink offset-y=1080 plane_id=1 &