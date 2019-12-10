#!/bin/bash

if [ -z $(docker ps -a --format "{{.Names}}" | grep "followbot") ]; then
    if [ -n "$IS_RPI" ]; then
	docker run -it \
	    --rm \
            --network=host \
  	    --privileged \
	    -w "/home/bot/followbot" \
	    -v "/dev/:/dev/:rw" \
	    -v "/media/$USER/:/media/$USER/:rw" \
	    -v "followbot-volume:/home/bot/followbot" \
	    --name=followbot thearchitector/followbot:raspi
    else
        docker run -it \
            --network=host \
  	    --privileged \
	    -e DISPLAY=$DISPLAY \
	    -v "/tmp/.X11-unix:/tmp/.X11-unix" \
	    -v "/dev:/dev" \
	    -v "followbot-volume:/home/developer/followbot" \
	    --name=followbot thearchitector/followbot:dev bash
    fi
else
    docker start -ai followbot
fi
