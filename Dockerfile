# Start from a base Docker distribution (Ubuntu 18.04)
FROM arm32v7/ros:melodic-robot-bionic

RUN apt-get update && \
    # Install a whole bunch of dependencies
    apt-get -y --no-install-recommends install build-essential cmake git libgtk-3-dev vim \
    pkg-config libavcodec-dev libavformat-dev libswscale-dev libtbb2 libtbb-dev curl && \
    apt-get -y autoremove && \
    apt-get -y clean

# Build and install OpenCV
RUN mkdir /tmp/opencv && \
    git clone https://github.com/opencv/opencv.git /tmp/opencv/core && \
    git clone https://github.com/opencv/opencv_contrib.git /tmp/opencv/contrib && \
    cd /tmp/opencv/core && \
    mkdir build && \
    cd build && \
    cmake -DCMAKE_BUILD_TYPE=RELEASE -DINSTALL_C_EXAMPLES=OFF -DWITH_TBB=ON -DWITH_V4L=ON \
          -DWITH_QT=OFF -DWITH_OPENGL=OFF -DOPENCV_EXTRA_MODULES_PATH=../../contrib/modules \
          -DBUILD_EXAMPLES=OFF -DBUILD_DOCS=OFF -DBUILD_opencv_java=OFF -DBUILD_opencv_python=OFF \
          -DBUILD_opencv_apps=OFF -DBUILD_openev_legacy=OFF -DBUILD_opencv_python2=OFF \
          -DBUILD_PERF_TESTS=OFF -DBUILD_TESTS=OFF -DWITH_OPENMP=ON -DWITH_IPP=OFF -DWITH_CSTRIPES=ON \
          -DENABLE_NEON=ON -DENABLE_VFPV3=ON -DCMAKE_INSTALL_PREFIX=/usr/local \
	  -DCMAKE_CXX_FLAGS="-DTBB_USE_GCC_BUILTINS=1 -D_TBB_64BIT_ATOMICS=0" \
	  -DOPENCV_ENABLE_NONFREE=ON -DCMAKE_SHARED_LINKER_FLAGS="-latomic" .. && \
    # Compile with max number of cores
    make -j4 && \
    make install && \
    rm -rf /tmp/*

# Setup bash entry point
COPY ./ros_entrypoint.sh /ros_entrypoint.sh
RUN chmod +x /ros_entrypoint.sh

# Setup perimissions
RUN (echo "root:password" | chpasswd) && \
    groupadd followers && \
    # Setup usergroup and user with home directory
    useradd -g followers -ms /bin/bash bot
USER bot
WORKDIR /home/bot

# Git clone the repo and update ROS dependencies
RUN cd /home/bot && \
    git clone https://github.com/thearchitector/followbot.git && \
    rosdep update

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
