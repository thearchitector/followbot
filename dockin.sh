#!/bin/bash

if [ -z $(docker ps -a --format "{{.Names}}" | grep "followbot") ]; then
    xhost +local:
    docker run -ite DISPLAY=$DISPLAY --rm --network=host --privileged -v "/home/developer/followbot" -v "/dev/:/dev/:rw" -v "/media/$USER/:/media/$USER/:rw" --name=followbot thearchitector/followbot:dev
else
    docker start -ai followbot
fi