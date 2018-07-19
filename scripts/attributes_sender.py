#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy

from fiware_ros_turtlebot3_operator.attributes_sender import AttributesSender

NODE_NAME = 'attributes_sender'


def main():
    try:
        rospy.init_node(NODE_NAME)
        AttributesSender().start()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
