#!/bin/bash

gst-launch-1.0 udpsrc port=5001 ! application/x-rtp,encoding-name=H264,payload=96 ! rtph264depay ! queue ! h264parse ! nvv4l2decoder ! nvdrmvideosink offset-x=1920 plane_id=1 &
