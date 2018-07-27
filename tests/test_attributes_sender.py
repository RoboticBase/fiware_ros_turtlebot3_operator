# -*- coding: utf-8 -*-
import unittest
import sys

import rosunit

from mock import patch

import freezegun

from nav_msgs.msg import Odometry

from . import utils


sys.path.append('./mock')


class TestAttributesSender(unittest.TestCase):

    @patch('fiware_ros_turtlebot3_operator.attributes_sender.rospy')
    def test_init(self, mocked_rospy):
        from fiware_ros_turtlebot3_operator.attributes_sender import AttributesSender
        from fiware_ros_turtlebot3_bridge.msg import r_pos

        mocked_rospy.get_param.return_value = utils.get_params()

        with freezegun.freeze_time('2018-01-02T03:04:05.123456+09:00'):
            receiver = AttributesSender()
        mocked_rospy.Subscriber('/odom', Odometry, receiver._on_odom_receive, queue_size=1)
        mocked_rospy.Publisher('/turtlebot3_bridge/attrs', r_pos, queue_size=10)

        assert receiver._AttributesSender__prev_ms.microsecond == 123456

    @patch('fiware_ros_turtlebot3_operator.attributes_sender.rospy')
    def test_start(self, mocked_rospy):
        from fiware_ros_turtlebot3_operator.attributes_sender import AttributesSender

        mocked_rospy.get_param.return_value = utils.get_params()

        AttributesSender().start()
        mocked_rospy.spin.assert_called_once_with()

    @patch('fiware_ros_turtlebot3_operator.attributes_sender.euler_from_quaternion')
    @patch('fiware_ros_turtlebot3_operator.attributes_sender.rospy')
    def test__on_odom_receive_under_threshold(self, mocked_rospy, mocked_euler_from_quaternion):
        from fiware_ros_turtlebot3_operator.attributes_sender import AttributesSender

        mocked_rospy.get_param.return_value = utils.get_params()

        with freezegun.freeze_time('2018-01-02T03:04:05.123456+09:00'):
            receiver = AttributesSender()

        with freezegun.freeze_time('2018-01-02T03:04:05.123457+09:00'):
            receiver._on_odom_receive(Odometry())

        mocked_euler_from_quaternion.assert_not_called()
        receiver._AttributesSender__turtlebot3_attrs_pub.publish.assert_not_called()

        assert receiver._AttributesSender__prev_ms.microsecond == 123456

    @patch('fiware_ros_turtlebot3_operator.attributes_sender.euler_from_quaternion')
    @patch('fiware_ros_turtlebot3_operator.attributes_sender.rospy')
    def test__on_odom_receive_over_threshold(self, mocked_rospy, mocked_euler_from_quaternion):
        from fiware_ros_turtlebot3_operator.attributes_sender import AttributesSender

        mocked_rospy.get_param.return_value = utils.get_params()

        with freezegun.freeze_time('2018-01-02T03:04:05.123456+09:00'):
            receiver = AttributesSender()

        odom = Odometry()
        odom.pose.pose.position.x = 1.0
        odom.pose.pose.position.y = 1.1
        odom.pose.pose.position.z = 1.2
        mocked_euler_from_quaternion.return_value = (0.0, 0.0, 1.3)

        with freezegun.freeze_time('2018-01-02T03:04:05.223456+09:00'):
            with patch('fiware_ros_turtlebot3_operator.attributes_sender.r_pos') as mocked_r_pos:
                receiver._on_odom_receive(odom)

        mocked_euler_from_quaternion.assert_called_once()
        receiver._AttributesSender__turtlebot3_attrs_pub.publish.assert_called_once_with(mocked_r_pos.return_value)
        mocked_r_pos.assert_called_once_with(x=1.0, y=1.1, z=1.2, theta=1.3)

        assert receiver._AttributesSender__prev_ms.microsecond == 223456


if __name__ == '__main__':
    rosunit.unitrun('fiware_ros_turtlebot3_operator', 'test_attributes_sender', TestAttributesSender)
