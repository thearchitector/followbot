# Followbot&trade; Docker Image

## Prerequisites
- A RaspberryPi
- An internet connection
- Docker

To install Docker on a RaspberryPi, simply run `curl -sSL https://get.docker.com | sh`. Docker will download and install automatically through `apt`. Once installed, it can be removed via `apt remove docker-ce`.

## Installation
To run this Docker image, you need to be logged into an authenticated Docker Hub account. Once logged in, you can run the Docker container like any other. The following commands will prompt you for your username and password, and will then download (if not already cached) the image and launch into bash.

```sh
    $ docker login
    $ docker run -it thearchitector/followbot:master
```

## Rebuilding
It is not recommened that this Docker image is rebuilt manually or edited in any way, as it has been carefully tailored for Followbot. A manual rebuild should not be necessary as Docker Hub is configured to automatically rebuild the image whenever the base ROS Melodic Ubunutu Bionic (`ros-melodic-bionic`) is changed or whenever a change is pushed to Git.

It should be noted that rebuilding the Docker image consumes around ~5 GiB of root partition space, and compilation takes between 5-10 minutes depending on the hardware avaliable. To minimize the build Docker image size, the command below assumes the Docker dameon is running with an experimental feature set. To enable experimental mode, follow the instructions [here](https://stackoverflow.com/a/44346323).

In the event that a manual rebuild is still needed, run the following commands:

```sh
    $ git clone https://github.com/thearchitector/followbot.git ~/followbot
    $ cd ~/followbot
    $ docker build --compress --squash --force-rm --no-cache -t thearchitector/followbot:master .
```

To repush the image to Docker Hub, run `docker push thearchitector/followbot:master` after rebuilding the image.

## License
This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License. To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-sa/4.0/ or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
