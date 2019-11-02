# Followbot&trade; Docker Image

## Prerequisites
- A RaspberryPi
- An internet connection
- Docker

To install Docker on a RaspberryPi, simply run the `curl -sSL https://get.docker.com | sh`. Docker will download and install automatically through `apt`. Once installed, it can be removed via `apt remove docker-ce`.

## Installation
To run this Docker image, you need to be logged into a Docker Hub account that has collaboration access. Once logged in, you can run the Docker container like any other. The following commands will prompt you for your username and password, and then download (if not already cached) the image and launch into bash.

```sh
    $ docker login
    $ docker run -it thearchitector/followbot:master
```

## License
This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License. To view a copy of this license, visit http://creativecommons.org/licenses/by-nc-sa/4.0/ or send a letter to Creative Commons, PO Box 1866, Mountain View, CA 94042, USA.
