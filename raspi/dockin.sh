#!/bin/bash

xhost +local:
docker run -ite DISPLAY=$DISPLAY -e QT_GRAPHICSSYSTEM=native --net host thearchitector/followbot:dev
