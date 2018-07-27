#!/usr/bin/env python
# -*- coding: utf-8 -*-
import unittest
import sys

import rosunit

from mock import patch

from parameterized import parameterized, param

from fiware_ros_turtlebot3_operator.logging import getLogger


class TestGetLogger(unittest.TestCase):

    @parameterized.expand([
        param(logm='debugf', rosm='logdebug'),
        param(logm='infof', rosm='loginfo'),
        param(logm='warnf', rosm='logwarn'),
        param(logm='errorf', rosm='logerr'),
        param(logm='fatalf', rosm='logfatal'),
    ])
    @patch('fiware_ros_turtlebot3_operator.logging.rospy')
    def test_log_wo_params(self, mocked_rospy, logm, rosm):
        name = 'foo'
        message = 'test message'
        log_message = '[{name}:{caller}] {message}'.format(
            name=name,
            caller=self.__class__.__name__ + '.' + sys._getframe().f_code.co_name,
            message=message,
        )

        logger = getLogger(name)
        assert logger.name == name

        getattr(logger, logm)(message)
        getattr(mocked_rospy, rosm).assert_called_once_with(log_message)

    @parameterized.expand([
        param(logm='debugf', rosm='logdebug'),
        param(logm='infof', rosm='loginfo'),
        param(logm='warnf', rosm='logwarn'),
        param(logm='errorf', rosm='logerr'),
        param(logm='fatalf', rosm='logfatal'),
    ])
    @patch('fiware_ros_turtlebot3_operator.logging.rospy')
    def test_log_w_params(self, mocked_rospy, logm, rosm):
        name = 'foo'
        message = 'test message'
        arg0 = 'arg0'
        arg1 = 'arg1'
        kwargs0 = 'kwargs0'
        kwargs1 = 'kwargs1'
        log_message = '[{name}:{caller}] {message}, {arg1}, {kwargs0}, {arg0}, {kwargs1}'.format(
            name=name,
            caller=self.__class__.__name__ + '.' + sys._getframe().f_code.co_name,
            message=message,
            arg0=arg0,
            arg1=arg1,
            kwargs0=kwargs0,
            kwargs1=kwargs1,
        )

        logger = getLogger(name)
        assert logger.name == name

        getattr(logger, logm)(message + ', {1}, {kwargs0}, {0}, {kwargs1}', arg0, arg1, kwargs1=kwargs1, kwargs0=kwargs0)
        getattr(mocked_rospy, rosm).assert_called_once_with(log_message)


if __name__ == '__main__':
    rosunit.unitrun('fiware_ros_turtlebot3_operator', 'test_logging', TestGetLogger)
