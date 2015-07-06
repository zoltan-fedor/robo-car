#!/bin/bash

#
# This script will start the video streaming from video0 from this robo-car device
#

echo "Starting vlc streaming"

IP_ADDRESS="$(/sbin/ifconfig wlan2 | grep 'inet addr:' | cut -d: -f2 | awk '{ print $1}')"
echo "Wlan2 current IP Address: $IP_ADDRESS"

# find and kill all instances of vlc on the server
kill $(ps aux | grep '[v]lc' | awk '{print $2}')

# start new vlc streaming on the IP_ADDRESS
cvlc v4l2:///dev/video0:chroma=h264:width=1024:height=576 --live-caching=200 --sout '#standard{access=http,mux=ts,dst='$IP_ADDRESS':8080,name=stream,mime=video/ts}' -vvv --sout-mux-caching=200 --clock-jitter=200 &
