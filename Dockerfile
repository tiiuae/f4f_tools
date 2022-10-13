FROM ghcr.io/tiiuae/fog-ros-baseimage:builder-c51e8f8 AS builder

COPY . /main_ws/src/

# this:
# 1) builds the application
# 2) packages the application as .deb & writes it to build_output/
RUN /packaging/build.sh

#  ▲               runtime ──┐
#  └── build                 ▼

FROM ghcr.io/tiiuae/fog-ros-baseimage:sha-c51e8f8

COPY --from=builder /main_ws/src/build_output/ros-*-f4f-tools_*_amd64.deb /f4f-tools.deb

# need update because ROS people have a habit of removing old packages pretty fast
RUN dpkg -i /f4f-tools.deb && rm /f4f-tools.deb

WORKDIR /f4f-tools

# make all commands in /f4f-tools/* invocable without full path
ENV PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/f4f-tools

# these do some of the things $ source /opt/ros/ROS_DISTRO/setup.bash does. we try
# to do this as not to require a separate shell just to prepare for launching the actual tool from f4f-tools.
ENV PYTHONPATH=/opt/ros/galactic/lib/python3.8/site-packages
ENV LD_LIBRARY_PATH=/opt/ros/galactic/opt/yaml_cpp_vendor/lib:/opt/ros/galactic/lib/x86_64-linux-gnu:/opt/ros/galactic/lib

COPY config/tmux.conf /etc/tmux.conf
COPY scripts/ /f4f-tools/

COPY entrypoint.sh /entrypoint.sh
ENTRYPOINT /entrypoint.sh

