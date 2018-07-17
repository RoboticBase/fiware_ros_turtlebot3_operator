# -*- coding: utf-8 -*-
import copy
import math
from threading import Lock
from contextlib import contextmanager

import rospy
from tf.transformations import quaternion_about_axis, quaternion_multiply, euler_from_quaternion
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from turtlebot3_operator.logging import getLogger
logger = getLogger(__name__)


COMMAND_SUB = '/turtlebot3_bridge/command'
TURTLEBOT3_cmd = '/cmd_vel'
LINEAR_X = 0.2
ANGULAR_Z = 0.3

TURTLEBOT3_CMD_PUB = '/cmd_vel'

TURTLEBOT3_ODOM_SUB = '/odom'
ROT_THRESHOLD_RAD = 0.02

EDGE_LENGTH_METER = 0.5

ROS_RATE = 10


class CommandReceiver(object):
    def __init__(self):
        super(CommandReceiver, self).__init__()
        self.__command_sub = rospy.Subscriber(COMMAND_SUB, String, self._on_command_receive, queue_size=10)
        self.__turtlebot3_cmd_pub = rospy.Publisher(TURTLEBOT3_CMD_PUB, Twist, queue_size=10)
        self.__turtlebot3_odom_pub = rospy.Subscriber(TURTLEBOT3_ODOM_SUB, Odometry, self._on_odom_receive, queue_size=10)

        self.__current_odometry = None
        self.__is_moving = False
        self.__lock = Lock()

    def start(self):
        logger.infof('CommandReceiver start')
        rospy.spin()
        logger.infof('CommandReceiver finish')

    def _on_command_receive(self, msg):
        raw_command = msg.data
        logger.infof('received command={}', raw_command)

        start_odometry = copy.deepcopy(self.__current_odometry)
        if raw_command == 'circle':
            with self._moving():
                self._circling(start_odometry)
        elif raw_command == 'stop':
            with self.__lock:
                self.__is_moving = False
            logger.infof('stop moving')
        else:
            logger.warnf('unknown command received, command={}', raw_command)

    def _on_odom_receive(self, msg):
        self.__current_odometry = msg

    def _circling(self, start_odometry):
        logger.infof('start circling')

        start_qt = start_odometry.pose.pose.orientation
        start_zrot = euler_from_quaternion([start_qt.x, start_qt.y, start_qt.z, start_qt.w])[2]
        oposite_zrot = euler_from_quaternion(quaternion_multiply([start_qt.x, start_qt.y, start_qt.z, start_qt.w],
                                                                 quaternion_about_axis(math.pi, [0, 0, 1])))[2]
        path_to_oposite = False

        twist = Twist()
        twist.linear.x = LINEAR_X
        twist.angular.z = ANGULAR_Z

        r = rospy.Rate(ROS_RATE)
        while not rospy.is_shutdown() and self.__is_moving:
            current_qt = self.__current_odometry.pose.pose.orientation
            current_zrot = euler_from_quaternion([current_qt.x, current_qt.y, current_qt.z, current_qt.w])[2]
            if path_to_oposite and abs(current_zrot - start_zrot) < ROT_THRESHOLD_RAD:
                break
            else:
                self.__turtlebot3_cmd_pub.publish(twist)
                if not path_to_oposite and abs(current_zrot - oposite_zrot) < ROT_THRESHOLD_RAD:
                    logger.infof('pass the oposite position')
                    path_to_oposite = True
            r.sleep()
        logger.infof('stop circling')

    @contextmanager
    def _moving(self):
        try:
            with self.__lock:
                self.__is_moving = True
            yield
        finally:
            with self.__lock:
                self.__is_moving = False
