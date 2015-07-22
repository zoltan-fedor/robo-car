#!/bin/bash

#
# This script will start the video streaming from video0 from this robo-car device
#

echo "Starting vlc streaming"

IP_ADDRESS="$(/sbin/ifconfig wlan2 | grep 'inet addr:' | cut -d: -f2 | awk '{ print $1}')"
echo "Wlan2 current IP Address: $IP_ADDRESS"

# find and kill all instances of vlc on the server
kill $(ps aux | grep '[v]lc' | awk '{print $2}')

# sleep for 1 sec
sleep 1

###################
# front-camera

# start new vlc streaming on the IP_ADDRESS
cvlc v4l2:///dev/video0:chroma=h264:width=1024:height=576 --live-caching=100 --sout '#standard{access=http,mux=ts,dst='$IP_ADDRESS':8080,name=stream,mime=video/ts}' -vvv --sout-mux-caching=100 --clock-jitter=0 &

###################
# rear-camera

# sleep for 1 sec
sleep 1

# start the rear-camera
cvlc v4l2:///dev/video1:chroma=mjpeg:width=432:height=240 --live-caching=100 --no-audio --sout '#transcode{vcodec=MJPG,venc=ffmpeg{strict=1},width=432,height=240,fps=10}:standard{access=http,mux=mpjpeg,dst='$IP_ADDRESS':8081}' -vvv --sout-mux-caching=100 --clock-jitter=0 &
