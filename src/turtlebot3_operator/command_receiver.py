# -*- coding: utf-8 -*-

import rospy

from turtlebot3_operator.logging import getLogger
logger = getLogger(__name__)


class CommandReceiver(object):
    def start(self):
        logger.infof('CommandReceiver start')
        rospy.spin()
        logger.infof('CommandReceiver finish')
