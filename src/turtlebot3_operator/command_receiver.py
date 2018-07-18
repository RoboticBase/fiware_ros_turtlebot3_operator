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

EDGE_LENGTH_METER = 1.0

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

        if not self.__current_odometry:
            logger.warnf('can not get current odometry')
            return

        if raw_command == 'circle':
            with self._moving():
                self._do_circle()
        elif raw_command == 'square':
            with self._moving():
                self._do_square()
        elif raw_command == 'triangle':
            with self._moving():
                self._do_triangle()
        elif raw_command == 'stop':
            with self.__lock:
                self.__is_moving = False
            logger.infof('stop moving')
        else:
            logger.warnf('unknown command received, command={}', raw_command)

    def _on_odom_receive(self, msg):
        self.__current_odometry = msg

    def _do_circle(self):
        logger.infof('start _do_circle')

        odom = copy.deepcopy(self.__current_odometry)

        start_qt = odom.pose.pose.orientation
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
                self.__turtlebot3_cmd_pub.publish(Twist())
                break
            else:
                self.__turtlebot3_cmd_pub.publish(twist)
                if not path_to_oposite and abs(current_zrot - oposite_zrot) < ROT_THRESHOLD_RAD:
                    logger.infof('pass the oposite position')
                    path_to_oposite = True
            r.sleep()
        logger.infof('stop _do_circle')

    def _do_square(self):
        logger.infof('start _do_square')

        start_odom = copy.deepcopy(self.__current_odometry)
        self._forward(start_odom, EDGE_LENGTH_METER)
        self._rotate(start_odom, math.pi/2)
        odom = copy.deepcopy(self.__current_odometry)
        self._forward(odom, EDGE_LENGTH_METER)
        self._rotate(start_odom, math.pi)
        odom = copy.deepcopy(self.__current_odometry)
        self._forward(odom, EDGE_LENGTH_METER)
        self._rotate(start_odom, math.pi * 3/2)
        odom = copy.deepcopy(self.__current_odometry)
        self._forward(odom, EDGE_LENGTH_METER)
        self._rotate(start_odom, math.pi * 2)

        logger.infof('stop _do_square')

    def _do_triangle(self):
        logger.infof('start _do_triangle')

        start_odom = copy.deepcopy(self.__current_odometry)
        self._forward(start_odom, EDGE_LENGTH_METER)
        self._rotate(start_odom, math.pi * 2/3)
        odom = copy.deepcopy(self.__current_odometry)
        self._forward(odom, EDGE_LENGTH_METER)
        self._rotate(start_odom, math.pi * 4/3)
        odom = copy.deepcopy(self.__current_odometry)
        self._forward(odom, EDGE_LENGTH_METER)
        self._rotate(start_odom, math.pi * 2)

        logger.infof('stop _do_triangle')

    def _forward(self, start_odometry, dist):
        logger.infof('start _forward, dist={}', dist)

        start_pos = start_odometry.pose.pose.position

        twist = Twist()
        twist.linear.x = LINEAR_X

        r = rospy.Rate(ROS_RATE)
        while not rospy.is_shutdown() and self.__is_moving:
            current_pos = self.__current_odometry.pose.pose.position
            if math.pow(start_pos.x - current_pos.x, 2) + math.pow(start_pos.y - current_pos.y, 2) >= math.pow(dist, 2):
                self.__turtlebot3_cmd_pub.publish(Twist())
                break
            else:
                self.__turtlebot3_cmd_pub.publish(twist)
            r.sleep()

        logger.infof('stop _forward')

    def _rotate(self, start_odometry, angle):
        logger.infof('start _rotate, angle={}', angle)

        start_qt = start_odometry.pose.pose.orientation
        target_zrot = euler_from_quaternion(quaternion_multiply([start_qt.x, start_qt.y, start_qt.z, start_qt.w],
                                                                quaternion_about_axis(angle, [0, 0, 1])))[2]

        twist = Twist()
        twist.angular.z = ANGULAR_Z

        r = rospy.Rate(ROS_RATE)
        while not rospy.is_shutdown() and self.__is_moving:
            current_qt = self.__current_odometry.pose.pose.orientation
            current_zrot = euler_from_quaternion([current_qt.x, current_qt.y, current_qt.z, current_qt.w])[2]
            if abs(current_zrot - target_zrot) < ROT_THRESHOLD_RAD:
                self.__turtlebot3_cmd_pub.publish(Twist())
                break
            else:
                self.__turtlebot3_cmd_pub.publish(twist)
            r.sleep()
        logger.infof('stop _rotate')

    @contextmanager
    def _moving(self):
        try:
            with self.__lock:
                self.__is_moving = True
            yield
        finally:
            with self.__lock:
                self.__is_moving = False
