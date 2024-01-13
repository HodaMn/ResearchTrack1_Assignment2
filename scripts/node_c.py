#! /usr/bin/env python3

import rospy
import actionlib
import actionlib.msg
#import assignment_2_2023.msg
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Twist, PoseStamped
from assignment_2_2023.msg import Info
from assignment_2_2023.srv import target

global avg_vx, avg_vy, n_samp

def clbk_info(msg):
    """
    Callback function for the '/bot_info' topic subscriber.
    Calculates the average velocity and distance from the current position to the target position.
    """
    global rx, ry, avg_vx, avg_vy, n_samp, distance

    distance = math.sqrt(pow((rx - msg.x), 2) + pow((ry - msg.x), 2))
    n_samp = n_samp + 1
    avg_vx = (avg_vx + msg.vel_x) / n_samp
    avg_vy = (avg_vy + msg.vel_y) / n_samp

def clbk_tgt(msg):
    """
    Callback function for the '/tgt' topic subscriber.
    Updates the target position.
    """
    global rx, ry
    rx = msg.x
    ry = msg.y

def main():
    global rx, ry, n_samp, avg_vx, avg_vy, distance

    n_samp = 0
    rx = 0
    ry = 0
    avg_vx = 0
    avg_vy = 0
    distance = 0

    # Init node
    rospy.init_node('node_c')

    # Subscribe to /bot_info
    sub_info = rospy.Subscriber('/bot_info', Info, clbk_info)
    sub_tgt = rospy.Subscriber('/tgt', Point, clbk_tgt)

    while not rospy.is_shutdown():
        print("Average Vx: ", float(f'{avg_vx:.6f}'), " Vy:", float(f'{avg_vy:.6f}'), "distance: ",
              float(f'{distance:.3f}'))
        rospy.sleep(1)  # Using rospy.sleep with no specified duration defaults to the rate of the rospy loop

if __name__ == "__main__":
    main()

