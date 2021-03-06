#!/usr/bin/env python
"""Test displaying the robot's current pose.

Before using this, run:
roslaunch subtle_launch static-sim.launch
roslaunch controller run.launch
"""

import time
import threading
import sys

import rospy

from controller_handlers.pose import poseHandler


def display_pose(pose_handler):
    """Print the current pose periodically."""
    print "",
    while not rospy.is_shutdown():
        pose = pose_handler.getPose()
        location = pose_handler.get_location()
        print "\rLocation: {}, Pose: {}".format(location, pose),
        sys.stdout.flush()
        time.sleep(1.0)


def main():
    """Get rooms from the user and send the robot there."""
    print "Initializing pose handler..."
    pose_handler = poseHandler(None, None, True)
    # Sleep to make sure the node has settled
    time.sleep(3.0)
    # Kick off display thread and spin
    threading.Thread(target=display_pose, args=(pose_handler,)).start()
    rospy.spin()


if __name__ == "__main__":
    main()
