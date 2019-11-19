#!/bin/bash
# Run the calibration executable on live camera
./cmake-build-debug/calibrate -w=9 -h=6 -s=0.023 -o=camera.yml \
 -oe -d=10 -dt=0.1845 -n=20 -p=chessboard