#!/usr/bin/env python
"""Test moving the robot around and getting its location.

Before using this, run:
roslaunch subtle_launch static-sim.launch
roslaunch controller run.launch
"""

import time

from controller_handlers.motion import MotionController


def main():
    """Get rooms from the user and send the robot there."""
    print "Initializing motion controller..."
    motion = MotionController(True)
    # Sleep to make sure the node has settled
    time.sleep(3.0)

    print "Enter the name of a room to go to, or ctrl-d to quit."
    print "The robot is in {!r}.".format(motion.get_location())
    try:
        while True:
            room = raw_input("> ").strip()
            if not room:
                print "The robot is in {!r}.".format(motion.get_location())
                continue

            # Send the command out
            result = motion.send_move_command(room)
            if not result:
                print "Could not find room {!r}.".format(room)
                continue
            else:
                print "The robot is in {!r} and is moving to {!r}.".format(
                    motion.get_location(), room)

    except (KeyboardInterrupt, EOFError):
        pass


if __name__ == "__main__":
    main()
