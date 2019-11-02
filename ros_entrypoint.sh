#!/bin/bash

source /opt/ros/melodic/setup.bash
cd ~/followbot && git checkout master -f && git pull
echo
exec "$@"
