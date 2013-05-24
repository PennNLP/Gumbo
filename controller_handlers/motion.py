"""Controller for LTLMoP motion handler for the JR platform."""

import roslib
roslib.load_manifest('controller_handlers')
import rospy

import actionlib
from geometry_msgs.msg import Pose
from robot_actions.msg import DriveToAction, DriveToGoal
from subtle_msgs.srv import GetTopoMap


class MotionController(object):
    """Send movement messages to the robot controller."""

    NODE_NAME = 'motion_controller'

    def __init__(self, init_node=False):
        self._name = type(self).__name__

        # Create our own node, but only if the caller requests it.
        if init_node:
            rospy.init_node(self.NODE_NAME)

        # Get the map
        rospy.wait_for_service('/getTopoMap')
        get_topo_map = rospy.ServiceProxy('getTopoMap', GetTopoMap)
        self.map = get_topo_map().topo_map

        # Get a client for driving
        self._drive_client = \
            actionlib.SimpleActionClient('drive_to', DriveToAction)
        self._drive_client.wait_for_server(rospy.Duration(5.0))
        self._drive_goal = None

    def send_move_command(self, room):
        """Move to the given room, returning whether the room exists."""
        target_position = _room_to_center(room, self.map)
        if not target_position:
            return False

        # Send the request
        goal = DriveToGoal()
        goal.target_pose = Pose(position=target_position)
        self._drive_client.send_goal(goal)
        self._drive_goal = goal
        print "{}: Moving robot to {!r}.".format(self._name, room)
        return True

    def stop(self):
        """Attempt to stop motion."""
        # If there's no active goal, there's nothing to be done
        if self._drive_goal:
            self._drive_client.cancel_goal()
            self._drive_goal = None
            print "{}: Cancelled robot drive goal.".format(self._name)

def _room_to_center(room, topo_map):
    """Return the center of a room on the map."""
    for region in topo_map.regions:
        if region.name == room:
            return region.center
    return None
