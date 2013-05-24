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

import re
from threading import Lock

import numpy

from fiducial_msgs.msg import FiducialScanStamped


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

        # Store reference to proj and pose handler
        self._pose_handler = proj.h_instance['pose']
        self._proj = proj

        # Initialize lock, current objects sensed, and list of objects to ignore
        self._id_fiducials = dict()
        self._disabled_items = set()
        self._currently_sensed = set()
        self._sensor_lock = Lock()

        self._last_region = None  # keep track of region we were in when last polled

    def _set_sensors(self, msg):
        """Reads current sensor status from the robot."""
        # Note: This callback is only triggered when something is being seen
        with self._sensor_lock:
            # Treat our last updated region as the current one, even if it's slightly
            # out-of-date. If we query a fresh location, we might just add fids
            # that will be deleted at an imminent boundary
            current_region = self._last_region

            # We accumulate here because we want to persist objects within a region
            for fid in msg.scan.fiducials:
                # Keep a dict of all the fiducials we know about
                self._id_fiducials[fid.id] = fid

                # Check if the fid is in the current region
                if self._fid_in_region(fid, current_region):
                    if fid.id not in self._currently_sensed:
                        print "{}: Adding {!r} to sensed items.".format(self._name, fid.id)
                    self._currently_sensed.add(fid.id)
        
        # Tell the simgui to show this fiducial on the map
        # TODO: Should we assume static pose and not send every time?)
        position = fid.pose.position
        raw_pose = numpy.array([position.x, position.y, position.z])
        pose = self._proj.coordmap_lab2map(raw_pose)
        self._proj.executor.postEvent("FID", [fid.id, pose[0], pose[1]])
    
    def _fid_in_region(self, fid, current_region):
        """Return whether a fiducial is in the current region."""
        position = fid.pose.position
        raw_pose = numpy.array([position.x, position.y, position.z])
        pose = self._proj.coordmap_lab2map(raw_pose)
        fid_regions = [region for region in self._proj.rfi.regions if
                       region.name.lower() != "boundary" and region.objectContainsPoint(*pose)]
        if fid_regions:
            fid_region = fid_regions[0].name
            return fid_region == current_region
        else:
            # Hopefully, we should not hit this case
            return False

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
            self._disabled_items.add(item.id)

    def get_sensed_item(self, name):
        """Return a fiducial matching a name, None if there is no match."""
        for item in self._currently_sensed - self._disabled_items:
            if _clean_item_id(item) == name:
                return self._id_fiducials[item]
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
