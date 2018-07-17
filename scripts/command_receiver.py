#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy

from turtlebot3_operator.command_receiver import CommandReceiver

NODE_NAME = 'command_receiver'


def main():
    try:
        rospy.init_node(NODE_NAME)
        CommandReceiver().start()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
