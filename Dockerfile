# Start from a base Docker distribution (Ubuntu 18.04)
FROM ros:melodic-robot-bionic

RUN apt-get update && \
    # Install a whole bunch of dependencies
    apt-get -y --no-install-recommends install build-essential cmake git libgtk2.0-dev \
    pkg-config libavcodec-dev libavformat-dev libswscale-dev libtbb2 libtbb-dev
    

# Install Visual Studio Code
RUN curl -L "https://go.microsoft.com/fwlink/?LinkID=760868" -o /tmp/code.deb && \
    apt-get -y --no-install-recommends install /tmp/code.deb

# Install ROS Melodic
RUN apt-get -y --no-install-recommends install ros-melodic-mavros* ros-melodic-joy \
	python-rosinstall python-rosinstall-generator python-wstool && \
    apt-get -y autoremove && \
    apt-get -y clean
    
# Build and install OpenCV
RUN rm -rf /tmp/* && \
    mkdir /tmp/opencv && \
    git clone https://github.com/opencv/opencv.git /tmp/opencv/core && \
    git clone https://github.com/opencv/opencv_contrib.git /tmp/opencv/contrib && \
    cd /tmp/opencv/core && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_BUILD_TYPE=RELEASE -DINSTALL_C_EXAMPLES=OFF -DWITH_TBB=ON -DWITH_V4L=ON \
          -DWITH_QT=OFF -DWITH_OPENGL=OFF -DOPENCV_EXTRA_MODULES_PATH=../../contrib/modules \
          -DBUILD_EXAMPLES=OFF -DBUILD_DOCS=OFF -DBUILD_opencv_java=OFF -DBUILD_opencv_python=OFF \
          -DBUILD_opencv_apps=OFF -DBUILD_openev_legacy=OFF -DBUILD_opencv_python2=OFF \
          -DBUILD_PERF_TESTS=OFF -DBUILD_TESTS=OFF -DWITH_OPENMP=ON -DWITH_IPP=ON -DWITH_CSTRIPES=ON \
          -DCMAKE_INSTALL_PREFIX=/usr/local .. && \
    # Compile with max number of cores - 1 to prevent lockup
    make -j$(nproc --ignore=1) && \
    make install

# Setup bash entry point
COPY ./ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

# Set root password
# TODO: Change
RUN (echo "root:password" | chpasswd) && \
    groupadd followers

# Setup usergroup and user with home directory
RUN useradd -g followers -ms /bin/bash bot
USER bot
WORKDIR /home/bot

# Git clone the repo and update ROS dependencies
RUN	cd /home/bot && \
	git clone https://github.com/thearchitector/followbot.git && \
	rosdep update

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
