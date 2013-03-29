#!/usr/bin/env python
"""LTLMoP pose handler for the JR platform.

Before using this, run:
roslaunch subtle_launch static-sim.launch
roslaunch controller run.launch
"""

import roslib
roslib.load_manifest('controller_handlers')
import rospy

from threading import Lock

import numpy

from geometry_msgs.msg import PoseStamped


class poseHandler:
    """Report the robot's current pose."""

    NODE_NAME = 'pose_controller'
    POSE_TOPIC = 'pose_publisher/pose'

    def __init__(self, proj, shared_data):
        self._name = type(self).__name__
        rospy.init_node(self.NODE_NAME)
        rospy.Subscriber(self.POSE_TOPIC, PoseStamped, self.set_pose)

        # Initialize pose and lock
        self._pose = numpy.array([0, 0, 0])
        self._pose_lock = Lock()

    def getPose(self):
        """Return the last reported pose."""
        with self._pose_lock:
            return self._pose

    def set_pose(self, msg):
        """Store the reported pose."""
        with self._pose_lock:
            position = msg.pose.position
            self._pose = numpy.array([position.x, position.y, position.z])
