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

from fiware_ros_turtlebot3_operator.logging import getLogger
logger = getLogger(__name__)


class CommandReceiver(object):
    def __init__(self):
        super(CommandReceiver, self).__init__()
        self.__params = rospy.get_param('~')
        rospy.Subscriber(self.__params['bridge']['topics']['cmd_sub'],
                         String,
                         self._on_command_receive,
                         queue_size=10)
        self.__turtlebot3_cmd_pub = rospy.Publisher(self.__params['turtlebot3']['topics']['cmd_pub'],
                                                    Twist,
                                                    queue_size=10)
        rospy.Subscriber(self.__params['turtlebot3']['topics']['odom_sub'],
                         Odometry,
                         self._on_odom_receive,
                         queue_size=10)

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
        elif raw_command == 'up':
            with self._moving():
                self._do_up()
        elif raw_command == 'down':
            with self._moving():
                self._do_down()
        elif raw_command == 'left':
            with self._moving():
                self._do_left()
        elif raw_command == 'right':
            with self._moving():
                self._do_right()
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
        start_pos = odom.pose.pose.position
        start_zrot = euler_from_quaternion([start_qt.x, start_qt.y, start_qt.z, start_qt.w])[2]
        moved_a_certain_distance = False

        twist = Twist()
        twist.linear.x = self.__params['turtlebot3']['circle']['velocities']['x']
        twist.angular.z = self.__params['turtlebot3']['circle']['velocities']['z']

        r = rospy.Rate(self.__params['turtlebot3']['rate_hz'])
        rot_threshold = self.__params['turtlebot3']['circle']['thresholds']['angular_rad']
        dist_threshold = self.__params['turtlebot3']['circle']['thresholds']['dist_meter']
        while not rospy.is_shutdown() and self.__is_moving:
            current_qt = self.__current_odometry.pose.pose.orientation
            current_zrot = euler_from_quaternion([current_qt.x, current_qt.y, current_qt.z, current_qt.w])[2]
            if moved_a_certain_distance and abs(current_zrot - start_zrot) < rot_threshold:
                break
            else:
                self.__turtlebot3_cmd_pub.publish(twist)
                current_pos = self.__current_odometry.pose.pose.position
                current_dist = math.sqrt(math.pow(start_pos.x - current_pos.x, 2) + math.pow(start_pos.y - current_pos.y, 2))

                if not moved_a_certain_distance and current_dist > dist_threshold:
                    logger.infof('moved a certain distance, current_dist={}', current_dist)
                    moved_a_certain_distance = True
            r.sleep()

        self.__turtlebot3_cmd_pub.publish(Twist())
        logger.infof('end _do_circle')

    def _do_square(self):
        logger.infof('start _do_square')

        edge_length = self.__params['turtlebot3']['polygon']['edge']['length_meter']
        start_odom = copy.deepcopy(self.__current_odometry)
        self._forward(start_odom, edge_length)
        self._rotate(start_odom, math.pi/2)
        odom = copy.deepcopy(self.__current_odometry)
        self._forward(odom, edge_length)
        self._rotate(start_odom, math.pi)
        odom = copy.deepcopy(self.__current_odometry)
        self._forward(odom, edge_length)
        self._rotate(start_odom, math.pi * 3/2)
        odom = copy.deepcopy(self.__current_odometry)
        self._forward(odom, edge_length)
        self._rotate(start_odom, math.pi * 2)

        logger.infof('end _do_square')

    def _do_triangle(self):
        logger.infof('start _do_triangle')

        edge_length = self.__params['turtlebot3']['polygon']['edge']['length_meter']
        start_odom = copy.deepcopy(self.__current_odometry)
        self._forward(start_odom, edge_length)
        self._rotate(start_odom, math.pi * 2/3)
        odom = copy.deepcopy(self.__current_odometry)
        self._forward(odom, edge_length)
        self._rotate(start_odom, math.pi * 4/3)
        odom = copy.deepcopy(self.__current_odometry)
        self._forward(odom, edge_length)
        self._rotate(start_odom, math.pi * 2)

        logger.infof('end _do_triangle')

    def _do_up(self):
        logger.infof('start _do_up')

        unit_length = self.__params['turtlebot3']['unit']['length_meter']
        start_odom = copy.deepcopy(self.__current_odometry)
        self._forward(start_odom, unit_length)

        logger.infof('end _do_up')

    def _do_down(self):
        logger.infof('start _do_down')

        unit_length = self.__params['turtlebot3']['unit']['length_meter']
        start_odom = copy.deepcopy(self.__current_odometry)
        self._forward(start_odom, unit_length, reverse=True)

        logger.infof('end _do_down')

    def _do_left(self):
        logger.infof('start _do_left')

        unit_theta = self.__params['turtlebot3']['unit']['theta_rad']
        start_odom = copy.deepcopy(self.__current_odometry)
        self._rotate(start_odom, unit_theta)

        logger.infof('end _do_left')

    def _do_right(self):
        logger.infof('start _do_right')

        unit_theta = self.__params['turtlebot3']['unit']['theta_rad']
        start_odom = copy.deepcopy(self.__current_odometry)
        self._rotate(start_odom, -1 * unit_theta, reverse=True)

        logger.infof('end _do_right')

    def _forward(self, start_odometry, dist, reverse=False):
        logger.infof('start _forward, dist={}', dist)

        start_pos = start_odometry.pose.pose.position

        twist = Twist()
        twist.linear.x = self.__params['turtlebot3']['polygon']['velocities']['x']
        if reverse:
            twist.linear.x *= -1

        r = rospy.Rate(self.__params['turtlebot3']['rate_hz'])
        while not rospy.is_shutdown() and self.__is_moving:
            current_pos = self.__current_odometry.pose.pose.position
            if math.pow(start_pos.x - current_pos.x, 2) + math.pow(start_pos.y - current_pos.y, 2) >= math.pow(dist, 2):
                break
            else:
                self.__turtlebot3_cmd_pub.publish(twist)
            r.sleep()

        self.__turtlebot3_cmd_pub.publish(Twist())
        logger.infof('end _forward')

    def _rotate(self, start_odometry, angle, reverse=False):
        logger.infof('start _rotate, angle={}', angle)

        start_qt = start_odometry.pose.pose.orientation
        target_zrot = euler_from_quaternion(quaternion_multiply([start_qt.x, start_qt.y, start_qt.z, start_qt.w],
                                                                quaternion_about_axis(angle, [0, 0, 1])))[2]

        twist = Twist()
        twist.angular.z = self.__params['turtlebot3']['polygon']['velocities']['z']
        if reverse:
            twist.angular.z *= -1

        r = rospy.Rate(self.__params['turtlebot3']['rate_hz'])
        rot_threshold = self.__params['turtlebot3']['polygon']['thresholds']['angular_rad']
        while not rospy.is_shutdown() and self.__is_moving:
            current_qt = self.__current_odometry.pose.pose.orientation
            current_zrot = euler_from_quaternion([current_qt.x, current_qt.y, current_qt.z, current_qt.w])[2]
            if abs(current_zrot - target_zrot) < rot_threshold:
                break
            else:
                self.__turtlebot3_cmd_pub.publish(twist)
            r.sleep()

        self.__turtlebot3_cmd_pub.publish(Twist())
        logger.infof('end _rotate')

    @contextmanager
    def _moving(self):
        try:
            with self.__lock:
                self.__is_moving = True
            yield
        finally:
            with self.__lock:
                self.__is_moving = False
