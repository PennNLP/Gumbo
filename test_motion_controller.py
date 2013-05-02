#!/usr/bin/env python
"""Test moving the robot around and getting its location.

Before using this, run:
roslaunch subtle_launch static-sim.launch
roslaunch controller run.launch
"""

import time
import threading

import rospy

from controller_handlers.motion import MotionController


def display_interface(motion):
    """Ask the user for locations and send the robot there."""
    print "Enter the name of a room to go to, or ctrl-d to quit."
    while not rospy.is_shutdown():
        room = raw_input("> ").strip()
        if not room:
            continue

        # Send the command out
        result = motion.send_move_command(room)
        if not result:
            print "Could not find room {!r}.".format(room)
            continue
        else:
            print "The robot is  moving to {!r}.".format(room)


def main():
    """Get rooms from the user and send the robot there."""
    print "Initializing motion controller..."
    motion = MotionController(True)
    # Sleep to make sure the node has settled
    time.sleep(3.0)

    # Kick off interface thread and spin
    threading.Thread(target=display_interface, args=(motion,)).start()
    rospy.spin()


if __name__ == "__main__":
    main()
