# Followbot&trade; Sensor and Processing RPi Controller

## Development
When developing, it is highly recommended you use the pre-built Docker image. Assuming Docker is already installed, it can be launched by running `./dockin.sh`.

`cmake` options:
```
-DCATKIN_DEVEL_PREFIX:PATH=/home/developer/followbot/raspi/followbot_ws/devel
```

`cmake` generation path:
```shell script
/home/developer/followbot/raspi/followbot_ws/build
```

If you're using Clion in the docker image, make sure to add these parameters to your build settings. 

To build, run from the `followbot_ws` (this) directory:
```shell script
catkin_make
source devel/setup.bash
```

Run the nodes:
```shell script
roscore & rosrun followbot loc_and_pc
```