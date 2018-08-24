FROM ros:kinetic
MAINTAINER Nobuyuki Matsui <nobuyuki.matsui@gmail.com>

ENV PYTHONUNBUFFERED 1

COPY ./kube_entrypoint.sh /opt/kube_entrypoint.sh
COPY . /opt/ros_ws/src/fiware_ros_turtlebot3_operator
WORKDIR /opt/ros_ws

RUN mv /bin/sh /bin/sh_tmp && ln -s /bin/bash /bin/sh
RUN apt update && apt upgrade -y && \
    apt install git python-pip -y && \
    apt-get install ros-kinetic-tf -y && \
    git clone https://github.com/tech-sketch/fiware_ros_turtlebot3_bridge.git src/fiware_ros_turtlebot3_bridge && \
    source /opt/ros/kinetic/setup.bash && \
    /opt/ros/kinetic/bin/catkin_make && \
    echo "source /opt/ros/kinetic/setup.bash" >> /root/.bashrc && \
    echo "source /opt/ros_ws/devel/setup.bash" >> /root/.bashrc
RUN rm /bin/sh && mv /bin/sh_tmp /bin/sh
