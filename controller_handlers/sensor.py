"""LTLMoP sensor handler for the JR platform.

Before using this, run:
roslaunch subtle_launch static-sim.launch

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

import re


class sensorHandler(object):
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

        # Store reference to pose handler
        self._pose_handler = proj.h_instance['pose']

        # Initialize lock, current objects sensed, and list of objects to ignore
        self._disabled_items = set()
        self._currently_sensed = set()
        self._sensor_lock = Lock()

        self._last_region = None  # keep track of region we were in when last polled

    def _set_sensors(self, msg):
        """Reads current sensor status from the robot."""

        # Note: This callback is only triggered when something is being seen

        with self._sensor_lock:
            # We accumulate here because we want to persist objects within a region
            # TODO: Filter out fiducials not in the current region
            self._currently_sensed |= set(fid for fid in msg.scan.fiducials)

    def get_sensor(self, sensor_name, initial=False):
        """Report whether we currently see a fiducial of the requested type.

        sensor_name (string): The type of the fiducial to query.
        """
        current_region = self._pose_handler.get_location()

        with self._sensor_lock:
            if initial:
                self._last_region = current_region
                # initialize landmark detection
                return True
            else:
                if current_region != self._last_region:
                    # We've entered a new region since the last poll;
                    # reset our accumulated list of sensed objects
                    print "{}: Resetting sensors on region change.".format(self._name)
                    self._currently_sensed = set()
                    self._last_region = current_region

                # Return whether we got an item back or none
                return self.get_sensed_item(sensor_name) is not None

    def disable_item(self, item):
        """Disable viewing of an item."""
        with self._sensor_lock:
            print "{}: Disabling sensing item {!r}.".format(self._name, item.id)
            self._disabled_items.add(item)

    def get_sensed_item(self, name):
        """Return an item matching a name, None if there is no match."""
        for item in self._currently_sensed - self._disabled_items:
            if name == _clean_item_id(item.id):
                return item
        else:
            return None

    def _get_all_sensors(self):
        """Return all types currently seen.

        To be used for testing only.
        """
        with self._sensor_lock:
            return list(self._currently_sensed)


def _clean_item_id(name):
    """Clean off any trailing numbers from an item's name."""
    return re.sub(r"\d+$", "", name)
