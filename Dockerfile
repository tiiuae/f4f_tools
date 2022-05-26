FROM ghcr.io/tiiuae/fog-ros-baseimage:sha-1cabd43

# pyserial + pymavlink are dependencies of mavlink_shell.
# unfortunately gcc is required to install pymavlink.
RUN apt-get update -y && apt-get install -y --no-install-recommends \
    python3-pip python3-systemd gcc iperf3\
    && rm -rf /var/lib/apt/lists/* \
    && pip3 install pyserial pymavlink mavsdk iperf3

WORKDIR /fog-tools

# Install pip and python dependencies
# RUN python3 -m pip install systemd

# make all commands in /fog-tools/* invocable without full path
ENV PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/fog-tools

# these do some of the things $ source /opt/ros/ROS_DISTRO/setup.bash does. we try
# to do this as not to require a separate shell just to prepare for launching the actual tool from fog-tools.
ENV PYTHONPATH=/opt/ros/galactic/lib/python3.8/site-packages
ENV LD_LIBRARY_PATH=/opt/ros/galactic/opt/yaml_cpp_vendor/lib:/opt/ros/galactic/lib/x86_64-linux-gnu:/opt/ros/galactic/lib

COPY src/ /fog-tools/

WORKDIR /tools-data

CMD ["ls", "-1", "/fog-tools"]
