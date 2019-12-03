#!/bin/bash

docker pull thearchitector/followbot:dev

if [ -z $(docker ps -a --format "{{.Names}}" | grep "followbot") ]; then
    #xhost +local:
    docker run -it \
    	--network=host \
	--privileged \
	-w "/home/developer/followbot" \
	-e DISPLAY=$DISPLAY \
	-e HOME="/home/developer" \
	-v "/tmp/.X11-unix:/tmp/.X11-unix" \
	-v "/dev/:/dev/:rw" \
	-v "/media/$USER/:/media/$USER/:rw" \
	-v "followbot-volume:/home/developer/followbot" \
	--name=followbot thearchitector/followbot:dev 
else
    docker start -ai followbot
fi
