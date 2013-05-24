"""LTLMoP motion handler for the JR platform."""

import roslib
roslib.load_manifest('controller_handlers')
import rospy

import sys

import actionlib
from geometry_msgs.msg import Pose
from robot_actions.msg import SweepAreaAction, SweepAreaGoal, DriveToAction, DriveToGoal


# Robot name
# TODO: Get this dynamically from LTLMoP
ROBOT_NAME = "ATRV_JR"
# Time it takes to defuse a bomb, in seconds
DEFUSE_TIME = 5.0


class gumboActuatorHandler(object):
    """Send actuation commands to the robot."""

    def __init__(self, proj, shared_data):  # pylint: disable=W0613
        self._name = type(self).__name__
        self._sensor_handler = proj.h_instance['sensor'][ROBOT_NAME]

        # Get a client for each action
        # Sweep
        self._sweep_client = actionlib.SimpleActionClient('sweep_area_action', SweepAreaAction)
        self._sweep_client.wait_for_server(rospy.Duration(5.0))
        self._sweep_goal = None

        # Defuse
        self._defuse_client = actionlib.SimpleActionClient('drive_to', DriveToAction)
        self._defuse_client.wait_for_server(rospy.Duration(5.0))
        self._defuse_goal = None

    def sweep(self, actuatorVal, initial=False):
        """Perform a search of the current room."""
        actuatorVal = _normalize(actuatorVal)

        # Activate or deactivate sweep
        if actuatorVal:
            print "{}: Activating sweep.".format(self._name)
            self._sweep_goal = SweepAreaGoal()
            self._sweep_goal.timeout = 150.0
            self._sweep_goal.wall_dist = 1.3
            self._sweep_goal.pattern = SweepAreaGoal.PATTERN_WALL_FOLLOW
            self._sweep_goal.args.append(SweepAreaGoal.ARG_RIGHT)
            self._sweep_client.send_goal(self._sweep_goal)
        else:
            print "{}: Deactivating sweep.".format(self._name)
            if self._sweep_goal:
                self._sweep_client.cancel_goal()
                self._sweep_goal = None

    def defuse(self, actuatorVal, initial=False):
        """Defuse a bomb by driving to it and making it disappear."""
        actuatorVal = _normalize(actuatorVal)
        
        # Activate or deactivate defuse
        if actuatorVal:
            # Get the bomb from the sensors
            bomb = self._sensor_handler.get_sensed_item("bomb")
            if not bomb:
                print >> sys.stderr, "{}: Defuse requested but no bomb found.".format(self._name)
                return

            # Move the robot to the bomb
            self._defuse_goal = DriveToGoal()
            bomb_position = bomb.pose.position
            self._defuse_goal.target_pose = Pose(position=bomb_position)

            # Sneakily define a lexically enclosed callback
            def _complete_defuse(goal_status, goal_result):
                """Wait until bomb is defused and then remove it from the sensors."""
                # TODO: Look at the status/result
                # Wait for the robot to reach the bomb, then pretend to
                # defuse the bomb by waiting then making it go away
                print "{}: Defusing bomb...".format(self._name)
                rospy.sleep(DEFUSE_TIME)
                print "{}: Bomb defusing complete.".format(self._name)
                self._sensor_handler.disable_item(bomb)

            self._defuse_client.send_goal(self._defuse_goal, done_cb=_complete_defuse)
            print "{}: Moving to bomb at ({}, {}).".format(self._name, bomb_position.x,
                                                          bomb_position.y)
        else:
            print "{}: Deactivating defuse.".format(self._name)
            if self._defuse_goal:
                self._defuse_client.cancel_goal()
                self._defuse_goal = None


def _normalize(value):
    """Normalize the value that an actuator is being set to."""
    # If it's not a boolean, it will be a string of an int.
    if not isinstance(value, bool):
        return bool(int(value))
    else:
        return value
