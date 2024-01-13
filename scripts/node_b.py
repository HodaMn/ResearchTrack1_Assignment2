#! /usr/bin/env python3

import rospy
#import assignment_2_2023.msg
from geometry_msgs.msg import Point
from assignment_2_2023.srv import target

def getInfoGoal():
    """
    Gets the last target coordinates from parameters and prints them.
    """
    # Get the last target coordinates from parameters
    last_target_x = rospy.get_param('/last_target_x', default=None)
    last_target_y = rospy.get_param('/last_target_y', default=None)

    if last_target_x is not None and last_target_y is not None:
        print("Last target coordinates: x = {}, y = {}".format(last_target_x, last_target_y))
    else:
        print("No last target coordinates available.")
        print("All parameters on the parameter server:")
        params = rospy.get_param_names()
        for param in params:
            value = rospy.get_param(param)
            print("{}: {}".format(param, value))

if __name__ == "__main__":
    rospy.init_node('get_info_goal_node')

    # Call the function to get and print the last target coordinates
    getInfoGoal()


