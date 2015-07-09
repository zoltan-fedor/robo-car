#!/bin/bash

SERVER="robo-car.int.thomsonreuters.com" # the server on which we execute the video restart restart
USER="car-control"

echo "Restarting vlc restreaming on $SERVER"

ssh $USER@$SERVER "echo \$HOME"

# find and kill all instances of vlc on the server
ssh $USER@$SERVER "kill \$(ps aux | grep '[v]lc' | awk '{print \$2}')"

# get the IP address of the robo-car
IP_ADDRESS="$(/sbin/ifconfig wlan2 | grep 'inet addr:' | cut -d: -f2 | awk '{ print $1}')"
echo "Wlan2 current IP Address on robo-car: $IP_ADDRESS"

# get the IP address of the server
IP_ADDRESS_SERVER="$(host $SERVER | awk '/has address/ { print $4 }')"
echo "Current IP Address of the server: $IP_ADDRESS_SERVER"

# start the vlc restream on the server
ssh $USER@$SERVER "cvlc http://$IP_ADDRESS:8080 --no-audio --sout '#transcode{vcodec=MJPG,venc=ffmpeg{strict=1},fps=15}:standard{access=http{mime=multipart/x-mixed-replace;boundary=--7b3cc56e5f51db803f790dad720ed50a},mux=mpjpeg,dst='$IP_ADDRESS_SERVER':8080}' --network-caching=100 "

