"""LTLMoP motion handler for the JR platform."""

import roslib
roslib.load_manifest('controller_handlers')
import rospy

import actionlib
from robot_actions.msg import SweepAreaAction


class gumboActuatorHandler(object):
    """Send actuation commands to the robot."""

    def __init__(self):
        self._name = type(self).__name__

        # Get the map
        rospy.wait_for_service('/getTopoMap')
        get_topo_map = rospy.ServiceProxy('getTopoMap', GetTopoMap)
        self.map = get_topo_map().topo_map

        # Get a client for each action
        self._sweep_client = actionlib.SimpleActionClient('sweep_area_action', SweepAreaAction)
        self._sweep_client.wait_for_server(rospy.Duration(5.0))
        self._sweep_goal = None

    def sweep(self, actuatorVal, initial=False):
        """Perform a search of the current room."""
        if actuatorVal:
            print "{}: Activating sweep.".format(self._name)
            self._sweep_goal = SweepAreaGoal()
            self._sweep_goal.timeout = 150.0
            self._sweep_goal.pattern = SweepAreaGoal.PATTERN_RESUME
            self._sweep_goal.args.append(SweepAreaGoal.ARG_RIGHT)
            self._sweep_client.send_goal(self._sweep_goal)
        else:
            print "{}: Deactivating sweep.".format(self._name)
            if self._sweep_goal:
                self._sweep_client.cancel_goal()
                self._sweep_goal = None
