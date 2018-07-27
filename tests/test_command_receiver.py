# -*- coding: utf-8 -*-
import unittest
import math

import rosunit

from mock import patch, MagicMock, PropertyMock, call

from parameterized import parameterized, param

from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry

from fiware_ros_turtlebot3_operator.command_receiver import CommandReceiver

from . import utils


class TestCommandReceiver(unittest.TestCase):

    @patch('fiware_ros_turtlebot3_operator.command_receiver.rospy')
    def test_init(self, mocked_rospy):
        mocked_rospy.get_param.return_value = utils.get_params()

        receiver = CommandReceiver()
        mocked_rospy.Subscriber('/turtlebot3_bridge/cmd', String, receiver._on_command_receive, queue_size=10)
        mocked_rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        mocked_rospy.Subscriber('/odom', Odometry, receiver._on_odom_receive, queue_size=10)

    @patch('fiware_ros_turtlebot3_operator.command_receiver.rospy')
    def test_start(self, mocked_rospy):
        mocked_rospy.get_param.return_value = utils.get_params()

        CommandReceiver().start()
        mocked_rospy.spin.assert_called_once_with()

    @patch('fiware_ros_turtlebot3_operator.command_receiver.rospy')
    def test__on_odom_receive(self, mocked_rospy):
        mocked_rospy.get_param.return_value = utils.get_params()

        odom = Odometry()

        receiver = CommandReceiver()
        receiver._on_odom_receive(odom)
        assert receiver._CommandReceiver__current_odometry == odom

    @patch('fiware_ros_turtlebot3_operator.command_receiver.rospy')
    def test__on_command_receive_wo_odometry(self, mocked_rospy):
        mocked_rospy.get_param.return_value = utils.get_params()

        msg = String(data='circle')
        receiver = CommandReceiver()
        receiver._move = MagicMock()

        receiver._on_command_receive(msg)
        receiver._move.assert_not_called()

    @parameterized.expand([
        param(cmd='circle', method='_do_circle'),
        param(cmd='square', method='_do_square'),
        param(cmd='triangle', method='_do_triangle'),
        param(cmd='up', method='_do_up'),
        param(cmd='down', method='_do_down'),
        param(cmd='left', method='_do_left'),
        param(cmd='right', method='_do_right'),
        param(cmd='stop', method=None),
        param(cmd='invalid', method=None),
    ])
    @patch('fiware_ros_turtlebot3_operator.command_receiver.rospy')
    def test__on_command_receive_w_odometry(self, mocked_rospy, cmd, method):
        mocked_rospy.get_param.return_value = utils.get_params()

        msg = String(data=cmd)
        receiver = CommandReceiver()
        receiver._move = MagicMock()

        odom = Odometry()
        receiver._CommandReceiver__current_odometry = odom
        receiver._CommandReceiver__is_moving = True

        receiver._on_command_receive(msg)
        if cmd in ('circle', 'square', 'triangle', 'up', 'down', 'left', 'right'):
            receiver._move.assert_called_once_with(getattr(receiver, method))
            assert receiver._CommandReceiver__is_moving is True
        elif cmd == 'stop':
            receiver._move.assert_not_called()
            assert receiver._CommandReceiver__is_moving is False
        else:
            receiver._move.assert_not_called()
            assert receiver._CommandReceiver__is_moving is True

    @patch('fiware_ros_turtlebot3_operator.command_receiver.euler_from_quaternion')
    @patch('fiware_ros_turtlebot3_operator.command_receiver.rospy')
    def test__do_circle(self, mocked_rospy, mocked_euler_from_quaternion):
        mocked_rospy.get_param.return_value = utils.get_params()
        mocked_rospy.is_shutdown.return_value = False

        receiver = CommandReceiver()
        receiver._CommandReceiver__is_moving = True

        mocked_odometry = MagicMock()

        counter = {'x': 0, 'y': 0, 'theta': 0}

        def mocked_x():
            if counter['x'] == 0:
                result = 0.0
            elif counter['x'] == 1:
                result = 0.0001
            elif counter['x'] == 2:
                result = 1.0
            elif counter['x'] == 3:
                result = 0.0001
            else:
                result = 0.0
            counter['x'] += 1
            return result

        def mocked_y():
            if counter['y'] == 0:
                result = 0.0
            elif counter['y'] == 1:
                result = 0.0001
            elif counter['y'] == 2:
                result = 1.0
            elif counter['y'] == 3:
                result = 0.0001
            else:
                result = 0.0
            counter['y'] += 1
            return result

        def mocked_theta(q):
            if counter['theta'] == 0:
                result = (0.0, 0.0, 0.0)
            elif counter['theta'] == 1:
                result = (0.0, 0.0, 0.0)
            elif counter['theta'] == 2:
                result = (0.0, 0.0, math.pi)
            elif counter['theta'] == 3:
                result = (0.0, 0.0, math.pi)
            else:
                result = (0.0, 0.0, 0.0)
            counter['theta'] += 1
            return result

        type(mocked_odometry.pose.pose.position).x = PropertyMock(side_effect=mocked_x)
        type(mocked_odometry.pose.pose.position).y = PropertyMock(side_effect=mocked_y)
        mocked_euler_from_quaternion.side_effect = mocked_theta

        receiver._CommandReceiver__current_odometry = mocked_odometry
        receiver._do_circle()
        twist_list = receiver._CommandReceiver__turtlebot3_cmd_pub.publish.call_args_list
        assert twist_list == [call(Twist(linear=Vector3(x=0.1), angular=Vector3(z=0.4))),
                              call(Twist(linear=Vector3(x=0.1), angular=Vector3(z=0.4))),
                              call(Twist(linear=Vector3(x=0.1), angular=Vector3(z=0.4))),
                              call(Twist())]
        mocked_rospy.Rate.assert_called_once_with(10)

    @patch('fiware_ros_turtlebot3_operator.command_receiver.rospy')
    def test__do_square(self, mocked_rospy):
        mocked_rospy.get_param.return_value = utils.get_params()
        receiver = CommandReceiver()

        mocked_forward = MagicMock()
        receiver._forward = mocked_forward
        mocked_rotate = MagicMock()
        receiver._rotate = mocked_rotate

        odom = Odometry

        receiver._CommandReceiver__current_odometry = odom

        receiver._do_square()
        assert mocked_forward.call_args_list == [call(odom, 0.4), call(odom, 0.4), call(odom, 0.4), call(odom, 0.4)]
        assert mocked_rotate.call_args_list == [call(odom, math.pi/2),
                                                call(odom, math.pi),
                                                call(odom, math.pi * 3/2),
                                                call(odom, math.pi * 2)]

    @patch('fiware_ros_turtlebot3_operator.command_receiver.rospy')
    def test__do_triangle(self, mocked_rospy):
        mocked_rospy.get_param.return_value = utils.get_params()
        receiver = CommandReceiver()

        mocked_forward = MagicMock()
        receiver._forward = mocked_forward
        mocked_rotate = MagicMock()
        receiver._rotate = mocked_rotate

        odom = Odometry

        receiver._CommandReceiver__current_odometry = odom

        receiver._do_triangle()
        assert mocked_forward.call_args_list == [call(odom, 0.4), call(odom, 0.4), call(odom, 0.4)]
        assert mocked_rotate.call_args_list == [call(odom, math.pi * 2/3),
                                                call(odom, math.pi * 4/3),
                                                call(odom, math.pi * 2)]

    @patch('fiware_ros_turtlebot3_operator.command_receiver.rospy')
    def test__do_up(self, mocked_rospy):
        mocked_rospy.get_param.return_value = utils.get_params()
        receiver = CommandReceiver()

        mocked_forward = MagicMock()
        receiver._forward = mocked_forward
        mocked_rotate = MagicMock()
        receiver._rotate = mocked_rotate

        odom = Odometry

        receiver._CommandReceiver__current_odometry = odom

        receiver._do_up()
        mocked_forward.assert_called_once_with(odom, 0.1)
        mocked_rotate.assert_not_called()

    @patch('fiware_ros_turtlebot3_operator.command_receiver.rospy')
    def test__do_down(self, mocked_rospy):
        mocked_rospy.get_param.return_value = utils.get_params()
        receiver = CommandReceiver()

        mocked_forward = MagicMock()
        receiver._forward = mocked_forward
        mocked_rotate = MagicMock()
        receiver._rotate = mocked_rotate

        odom = Odometry

        receiver._CommandReceiver__current_odometry = odom

        receiver._do_down()
        mocked_forward.assert_called_once_with(odom, 0.1, reverse=True)
        mocked_rotate.assert_not_called()

    @patch('fiware_ros_turtlebot3_operator.command_receiver.rospy')
    def test__do_left(self, mocked_rospy):
        mocked_rospy.get_param.return_value = utils.get_params()
        receiver = CommandReceiver()

        mocked_forward = MagicMock()
        receiver._forward = mocked_forward
        mocked_rotate = MagicMock()
        receiver._rotate = mocked_rotate

        odom = Odometry

        receiver._CommandReceiver__current_odometry = odom

        receiver._do_left()
        mocked_forward.assert_not_called()
        mocked_rotate.assert_called_once_with(odom, 0.01)

    @patch('fiware_ros_turtlebot3_operator.command_receiver.rospy')
    def test__do_right(self, mocked_rospy):
        mocked_rospy.get_param.return_value = utils.get_params()
        receiver = CommandReceiver()

        mocked_forward = MagicMock()
        receiver._forward = mocked_forward
        mocked_rotate = MagicMock()
        receiver._rotate = mocked_rotate

        odom = Odometry

        receiver._CommandReceiver__current_odometry = odom

        receiver._do_right()
        mocked_forward.assert_not_called()
        mocked_rotate.assert_called_once_with(odom, -1 * 0.01, reverse=True)

    @parameterized.expand([param(reverse=False), param(reverse=True)])
    @patch('fiware_ros_turtlebot3_operator.command_receiver.rospy')
    def test__do_forward(self, mocked_rospy, reverse):
        mocked_rospy.get_param.return_value = utils.get_params()
        mocked_rospy.is_shutdown.return_value = False

        receiver = CommandReceiver()
        receiver._CommandReceiver__is_moving = True

        mocked_odometry = MagicMock()

        counter = {'x': 0, 'y': 0}

        def mocked_x():
            if counter['x'] == 0:
                result = 0.0
            elif counter['x'] == 1:
                result = 0.0001
            elif counter['x'] == 2:
                result = 1.0
            elif counter['x'] == 3:
                result = 0.0001
            else:
                result = 0.0
            counter['x'] += 1
            return result

        def mocked_y():
            if counter['y'] == 0:
                result = 0.0
            elif counter['y'] == 1:
                result = 0.0001
            elif counter['y'] == 2:
                result = 1.0
            elif counter['y'] == 3:
                result = 0.0001
            else:
                result = 0.0
            counter['y'] += 1
            return result

        type(mocked_odometry.pose.pose.position).x = PropertyMock(side_effect=mocked_x)
        type(mocked_odometry.pose.pose.position).y = PropertyMock(side_effect=mocked_y)

        receiver._CommandReceiver__current_odometry = mocked_odometry
        receiver._forward(Odometry(), 1.0, reverse=reverse)
        twist_list = receiver._CommandReceiver__turtlebot3_cmd_pub.publish.call_args_list
        if reverse:
            assert twist_list == [call(Twist(linear=Vector3(x=-0.2))), call(Twist(linear=Vector3(x=-0.2))), call(Twist())]
        else:
            assert twist_list == [call(Twist(linear=Vector3(x=0.2))), call(Twist(linear=Vector3(x=0.2))), call(Twist())]

    @parameterized.expand([param(reverse=False), param(reverse=True)])
    @patch('fiware_ros_turtlebot3_operator.command_receiver.euler_from_quaternion')
    @patch('fiware_ros_turtlebot3_operator.command_receiver.rospy')
    def test__do_rotate(self, mocked_rospy, mocked_euler_from_quaternion, reverse):
        mocked_rospy.get_param.return_value = utils.get_params()
        mocked_rospy.is_shutdown.return_value = False

        receiver = CommandReceiver()
        receiver._CommandReceiver__is_moving = True

        mocked_odometry = MagicMock()

        counter = {'theta': 0}

        def mocked_theta(q):
            if counter['theta'] == 0:
                result = (0.0, 0.0, math.pi)
            elif counter['theta'] == 1:
                result = (0.0, 0.0, 0.0)
            elif counter['theta'] == 2:
                result = (0.0, 0.0, 0.001)
            elif counter['theta'] == 3:
                result = (0.0, 0.0, math.pi)
            else:
                result = (0.0, 0.0, 0.0)
            counter['theta'] += 1
            return result

        mocked_euler_from_quaternion.side_effect = mocked_theta

        receiver._CommandReceiver__current_odometry = mocked_odometry
        receiver._rotate(Odometry(), math.pi, reverse=reverse)
        twist_list = receiver._CommandReceiver__turtlebot3_cmd_pub.publish.call_args_list
        if reverse:
            assert twist_list == [call(Twist(angular=Vector3(z=-0.2))), call(Twist(angular=Vector3(z=-0.2))), call(Twist())]
        else:
            assert twist_list == [call(Twist(angular=Vector3(z=0.2))), call(Twist(angular=Vector3(z=0.2))), call(Twist())]

    @patch('fiware_ros_turtlebot3_operator.command_receiver.rospy')
    def test__move(self, mocked_rospy):
        mocked_rospy.get_param.return_value = utils.get_params()

        receiver = CommandReceiver()
        counter = type('Counter', (object,), {"count": 0})()

        def callback():
            counter.count += 1
            assert receiver._CommandReceiver__is_moving is True

        assert receiver._CommandReceiver__is_moving is False
        assert counter.count == 0
        t = receiver._move(callback)
        t.join()
        assert receiver._CommandReceiver__is_moving is False
        assert counter.count == 1

    @patch('fiware_ros_turtlebot3_operator.command_receiver.rospy')
    def test__move_when_moving(self, mocked_rospy):
        mocked_rospy.get_param.return_value = utils.get_params()

        receiver = CommandReceiver()
        receiver._CommandReceiver__is_moving = True
        counter = type('Counter', (object,), {"count": 0})()

        def callback():
            assert False
            counter.count += 1

        assert receiver._CommandReceiver__is_moving is True
        assert counter.count == 0
        t = receiver._move(callback)
        t.join()
        assert receiver._CommandReceiver__is_moving is True
        assert counter.count == 0


if __name__ == '__main__':
    rosunit.unitrun('fiware_ros_turtlebot3_operator', 'test_command_receiver', TestCommandReceiver)
