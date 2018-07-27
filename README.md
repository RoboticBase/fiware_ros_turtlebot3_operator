# fiware_ros_turtlebot3_operator
This ros package controls "[turtlebot3](http://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)" robot (both actual robot and simulator).

[![TravisCI Status](https://travis-ci.org/tech-sketch/fiware_ros_turtlebot3_operator.svg?branch=master)](https://travis-ci.org/tech-sketch/fiware_ros_turtlebot3_operator)

## Description
### `command_receiver`
This ROS node subscribes a ROS topic and receive a command from `fiware_ros_turtlebot3_bridge`. And this node publishes a serise of messages to `/cmd_vel` topic according to the received command in order to operate "turtlebot3".

### `attribute_sender`
This ROS node subscribes the `/odom` topic in order to receive the current position and current quaternion of "turtlebot3". When this node receives a new message, this node calculates the euler angle of z axis, and publishes a current position and angle to a ROS topic.


## Requirement

**ROS kinetic**

## Prepare
### configure parameters to operate "turtlebot3"

```bash
$ cp src/fiware_ros_turtlebot3_bridge/config/mqtt.yaml.template src/fiware_ros_turtlebot3_bridge/config/mqtt.yaml
$ vi src/fiware_ros_turtlebot3_bridge/config/mqtt.yaml
```

## How to Run

```bash
$ roslaunch fiware_ros_turtlebot3_operator fiware_ros_turtlebot3_operator.launch
```

## License

[Apache License 2.0](/LICENSE)

## Copyright
Copyright (c) 2018 TIS Inc.
