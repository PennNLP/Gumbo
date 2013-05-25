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
import time, logging

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

from tf.transformations import euler_from_quaternion

class poseHandler:
    """Report the robot's current pose."""

    NODE_NAME = 'pose_controller'
    POSE_TOPIC = 'pose_publisher/pose'
    LOCATION_TOPIC = 'location'

    def __init__(self, proj, shared_data, init_node=False):  # pylint: disable=W0613
        """
        init_node (bool): create separate ROS node (default: False)
        """
    
        self._name = type(self).__name__
        # Create our own node, but only if the caller requests it.
        if init_node:
            rospy.init_node(self.NODE_NAME)

        # Initialize pose and lock
        self._pose = numpy.array([0, 0, 0])
        self._pose_lock = Lock()

        # Initialize location and lock
        self._location = None
        self._location_lock = Lock()

        # Subscribe to location and pose updates
        rospy.Subscriber(self.POSE_TOPIC, PoseStamped, self.set_pose)
        rospy.Subscriber(self.LOCATION_TOPIC, String, self.set_location)

        # Wait for first location message
        logging.debug("Waiting for first location message...")
        while not self._location:
            time.sleep(0.1)

    def getPose(self, cached=True):  # pylint: disable=W0613
        """Return the last reported pose.

        The optional second argument is required by LTLMoP but we 
        ignore it.
        """
        with self._pose_lock:
            return self._pose

    def set_pose(self, msg):
        """Store the reported pose."""
        with self._pose_lock:
            position = msg.pose.position
            o = msg.pose.orientation # quaternion
            yaw = euler_from_quaternion([o.x, o.y, o.z, o.w])[2]
            self._pose = numpy.array([position.x, position.y, yaw])

    def get_location(self):
        """Return the room the robot is currently in."""
        with self._location_lock:
            return self._location

    def set_location(self, msg):
        """Set the location of the robot."""
        with self._location_lock:
            self._location = msg.data
