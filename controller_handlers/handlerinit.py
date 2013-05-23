"""Provides initialization for other Gumbo handlers."""

import roslib
roslib.load_manifest('controller_handlers')
import rospy


class gumboInitHandler:
    """Initialize a ROS node for the other handlers."""

    NODE_NAME = "gumbo_controller"

    def __init__(self, proj):  # pylint: disable=W0613
        rospy.init_node(self.NODE_NAME)

    def getSharedData(self):
        """Return a dict of objects shared with other handlers."""
        return {}

    def _close(self):
        """Shut down the ROS node."""
        rospy.signal_shutdown("Gumbo handler is being closed.")
