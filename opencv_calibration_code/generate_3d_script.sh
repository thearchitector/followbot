#!/bin/bash
touch placeholder.jpg
./cmake-build-debug/generate_3d --max-disparity=16 -i=camera.yml -e=camera.yml -o=disparity_image.jpg --blocksize=15
rm -rf placeholder.jpg