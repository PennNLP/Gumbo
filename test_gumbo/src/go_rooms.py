#!/usr/bin/python
"""
Interface for sending the robot from room to room.

Before using this, run:
roslaunch subtle_launch static-sim.launch
roslaunch controller run.launch
"""

import roslib
roslib.load_manifest('test_gumbo')
import rospy

import time

import actionlib
from geometry_msgs.msg import Pose
from robot_actions.msg import DriveToAction, DriveToGoal
from subtle_msgs.srv import GetTopoMap


def _room_to_center(room, topo_map):
    """Return the center of a room on the map."""
    for region in topo_map.regions:
        if region.name == room:
            return region.center
    return None


def main():
    """Get rooms from the user and send the robot there."""
    print "Initializing node..."
    rospy.init_node("test_gumbo_rooms")
    # Sleep to make sure the node has settled
    time.sleep(3.0)

    client = actionlib.SimpleActionClient('drive_to', DriveToAction)
    client.wait_for_server(rospy.Duration(5.0))
    rospy.wait_for_service('/getTopoMap')
    get_topo_map = rospy.ServiceProxy('getTopoMap', GetTopoMap)
    topo_response = get_topo_map()

    print "Enter the name of a room to go to:"
    try:
        while True:
            room = raw_input("> ").strip()
            if not room:
                break

            target_position = _room_to_center(room, topo_response.topo_map)
            if not target_position:
                print "Could not find room {!r}.".format(room)
                continue

            # Send the request
            goal = DriveToGoal()
            goal.target_pose = Pose(position=target_position)
            client.send_goal(goal)
            print "Moving robot to {!r}.".format(room)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
