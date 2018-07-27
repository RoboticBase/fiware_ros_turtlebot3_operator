FROM ros:kinetic
MAINTAINER Nobuyuki Matsui <nobuyuki.matsui@gmail.com>

COPY . /opt/ros_ws/src/fiware_ros_turtlebot3_operator
WORKDIR /opt/ros_ws

RUN mv /bin/sh /bin/sh_tmp && ln -s /bin/bash /bin/sh
RUN apt update && apt upgrade -y && \
    apt install python-pip -y && \
    apt-get install ros-kinetic-tf -y && \
    source /opt/ros/kinetic/setup.bash && \
    /opt/ros/kinetic/bin/catkin_make && \
    echo "source /opt/ros/kinetic/setup.bash" >> /root/.bashrc && \
    echo "source /opt/ros_ws/devel/setup.bash" >> /root/.bashrc
RUN rm /bin/sh && mv /bin/sh_tmp /bin/sh
