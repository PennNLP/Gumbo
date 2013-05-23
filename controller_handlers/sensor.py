"""LTLMoP sensor handler for the JR platform.

Before using this, run:
roslaunch subtle_launch static-sim.launch
roslaunch controller run.launch

Mappings currently in use are from subtle_launch/launch/main.lauch:
      <rosparam param="id2name">
        {
        "195" : "bomb1",
        "131" : "bomb2",
        "139" : "badguy",
        "225" : "hostage",
        "169" : "user2"
        }
      </rosparam>

"""

import roslib
roslib.load_manifest('controller_handlers')
import rospy

from threading import Lock

from fiducial_msgs.msg import FiducialScanStamped


class sensorHandler:
    """Report the robot's current sensor status."""

    NODE_NAME = 'sensor_controller'
    SENSOR_TOPIC = 'fiducial_scan'

    def __init__(self, proj, shared_data, init_node=False):  # pylint: disable=W0613
        """
        init_node (bool): create separate ROS node (default: False)
        """

        self._name = type(self).__name__
        # Create our own node, but only if the caller requests it.
        if init_node:
            rospy.init_node(self.NODE_NAME)
        rospy.Subscriber(self.SENSOR_TOPIC, FiducialScanStamped,
                         self._set_sensors)

        # Initialize lock and current objects sensed
        self._currently_sensed = set()
        self._sensor_lock = Lock()

    def _set_sensors(self, msg):
        """Reads current sensor status from the robot."""
        with self._sensor_lock:
            # TODO: Rather than clearing the set every time, we
            # need to clear it if the region has changed but keep
            # things in there if the region hasn't since last update.
            # TODO: Deactivate bombs on defusing
            self._currently_sensed = set(fid.id for fid in msg.scan.fiducials)

    def get_sensor(self, sensor_name, initial=False):
        """Report whether we currently see a fiducial of the requested type.

        sensor_name (string): The type of the fiducial to query.
        """
        with self._sensor_lock:
            if initial:
                # initialize landmark detection
                return True
            else:
                return sensor_name in self._currently_sensed

    def _get_all_sensors(self):
        """Return all types currently seen.

        To be used for testing only.
        """
        with self._sensor_lock:
            return list(self._currently_sensed)
