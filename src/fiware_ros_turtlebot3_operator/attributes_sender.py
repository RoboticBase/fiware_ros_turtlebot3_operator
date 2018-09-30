# -*- coding: utf-8 -*-
import datetime
from threading import Lock

import rospy
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry

from fiware_ros_turtlebot3_msgs.msg import r_pos

from fiware_ros_turtlebot3_operator.logging import getLogger
logger = getLogger(__name__)


class AttributesSender(object):
    def __init__(self):
        super(AttributesSender, self).__init__()
        self.__params = rospy.get_param('~')
        rospy.Subscriber(self.__params['turtlebot3']['topics']['odom_sub'],
                         Odometry,
                         self._on_odom_receive,
                         queue_size=1)
        self.__turtlebot3_attrs_pub = rospy.Publisher(self.__params['bridge']['topics']['attrs_pub'],
                                                      r_pos,
                                                      queue_size=10)

        self.__send_delta_ms = self.__params['bridge']['thresholds']['send_delta_millisec']
        self.__prev_ms = datetime.datetime.now()
        self.__lock = Lock()

    def start(self):
        logger.infof('AttributesSender start')
        rospy.spin()
        logger.infof('AttributesSender finish')

    def _on_odom_receive(self, odometry):
        now = datetime.datetime.now()
        if now >= self.__prev_ms + datetime.timedelta(milliseconds=self.__send_delta_ms) and self.__lock.acquire(False):
            self.__prev_ms = now

            pos = odometry.pose.pose.position
            qt = odometry.pose.pose.orientation
            theta = euler_from_quaternion([qt.x, qt.y, qt.z, qt.w])[2]

            msg = r_pos(x=pos.x, y=pos.y, z=pos.z, theta=theta)
            self.__turtlebot3_attrs_pub.publish(msg)

            self.__lock.release()
