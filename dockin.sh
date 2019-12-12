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
	XSOCK=/tmp/.X11-unix
	XAUTH=/tmp/.docker.xauth

	if [ ! -f $XAUTH ]; then
	    touch $XAUTH
	    xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
	fi

        docker run -it \
            --network=host \
  	    --privileged \
	    -e DISPLAY \
	    -e XAUTHORITY=$XAUTH \
	    -v $XSOCK:$XSOCK:rw \
	    -v $XAUTH:$XAUTH:rw \
	    -v "/dev:/dev" \
	    -v "followbot-volume:/home/developer/followbot" \
	    --name=followbot thearchitector/followbot:dev bash
    fi
else
    docker start -ai followbot
fi
