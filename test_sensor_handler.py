#!/usr/bin/env python
"""Test displaying the robot's current sensors.

Before using this, run:
roslaunch subtle_launch static-sim.launch
roslaunch controller run.launch
"""

import time
import threading

import rospy

from controller_handlers.sensor import sensorHandler


def display_sensors(sensor_handler):
    """Print the current sensor state periodically."""
    while not rospy.is_shutdown():
        # We use a testing hook to get all sensors at once.
        # pylint: disable=W0212
        sensors = sensor_handler._get_all_sensors()
        print "Sensors:", sensors
        time.sleep(0.5)


def main():
    """Get rooms from the user and send the robot there."""
    print "Initializing sensor handler..."
    sensor_handler = sensorHandler(None, None, True)
    # Sleep to make sure the node has settled
    time.sleep(3.0)
    # Kick off display thread and spin
    threading.Thread(target=display_sensors, args=(sensor_handler,)).start()
    rospy.spin()


if __name__ == "__main__":
    main()
