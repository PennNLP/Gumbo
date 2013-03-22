"""LTLMoP motion handler for the JR platform."""

import roslib
roslib.load_manifest('controller_handlers')
import rospy

from threading import Lock

import actionlib
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from robot_actions.msg import DriveToAction, DriveToGoal
from subtle_msgs.srv import GetTopoMap


HANDLER_NAME = 'MotionHandler'
CONTROLLER_NAME = 'MotionController'
NODE_NAME = 'motion_controller'
LOCATION_TOPIC = 'location'


class MotionController:
    """Send movement messages to the robot controller."""

    def __init__(self):
        rospy.init_node(NODE_NAME)

        # Get the map
        rospy.wait_for_service('/getTopoMap')
        get_topo_map = rospy.ServiceProxy('getTopoMap', GetTopoMap)
        self.map = get_topo_map().topo_map

        # Get a client for driving
        self.drive_client = \
            actionlib.SimpleActionClient('drive_to', DriveToAction)
        self.drive_client.wait_for_server(rospy.Duration(5.0))

        # Initialize location and lock
        self.location = None
        self.location_lock = Lock()

        # Subscribe to location updates
        rospy.Subscriber(LOCATION_TOPIC, String, self.set_location)

    def send_move_command(self, room):
        """Move to the given room, returning whether the room exists."""
        target_position = _room_to_center(room, self.map)
        if not target_position:
            return False

        # Send the request
        goal = DriveToGoal()
        goal.target_pose = Pose(position=target_position)
        self.drive_client.send_goal(goal)
        print "{}: Moving robot to {!r}.".format(CONTROLLER_NAME, room)
        return True

    def get_location(self):
        """Return the room the robot is currently in."""
        with self.location_lock:
            return self.location

    def set_location(self, msg):
        """Set the location of the robot."""
        with self.location_lock:
            self.location = msg.data


def _room_to_center(room, topo_map):
    """Return the center of a room on the map."""
    for region in topo_map.regions:
        if region.name == room:
            return region.center
    return None


class MotionHandler:
    """Send drive commands using MotionController."""

    def __init__(self):
        self.controller = MotionController()
        self.next_region = None

    def gotoRegion(self, current_region, next_region):
        """Try to drive to next_region, return whether we have arrived."""
        if current_region == next_region:
            # Reset next_region, and report that we are there already.
            self.next_region = None
            return True
        elif self.next_region == next_region:
            # We're already trying to go there. Check whether we've arrived.
            if self._at_destination():
                self.next_region = None
                print "{}: Arrived at destination {!r}.".format(
                    CONTROLLER_NAME, next_region)
                return True
            else:
                return False
        else:
            # Double check that we're not already there.
            if self._at_destination():
                # This should only occur in the rare case that the
                # robot has changed regions since FSA last checked.
                print "{}: Already at destination {!r}.".format(
                    CONTROLLER_NAME, next_region)
                return True
            else:
                print "{}: Moving from {!r} to {!r}.".format(
                    CONTROLLER_NAME, current_region, next_region)
                self.next_region = next_region
                self.controller.send_move_command(next_region)
                return False

    def _at_destination(self):
        """Return whether we have reached our destination."""
        return self.controller.get_location() == self.next_region