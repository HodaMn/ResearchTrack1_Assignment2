#!/usr/bin/env python3
import sys
import rospy
import actionlib
import time
import actionlib.msg
import assignment_2_2023.msg
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Twist, PoseStamped
from assignment_2_2023.msg import PlanningAction, PlanningGoal
from assignment_2_2023.msg import Info
from assignment_2_2023.srv import target

# Global variables
pose_ = Pose()
twist_ = Twist()

def clbk_odom(msg):
    """
    Callback function for '/odom' topic.
    Updates the position and velocity information.
    """
    global pub_info

    x_ = msg.pose.pose.position.x
    y_ = msg.pose.pose.position.y
    vx_ = msg.twist.twist.linear.x
    vy_ = msg.twist.twist.linear.y

    msg_info = Info()

    msg_info.x = x_
    msg_info.y = y_
    msg_info.vel_x = vx_
    msg_info.vel_y = vy_

    if not rospy.is_shutdown():
        pub_info.publish(msg_info)

def ltk_tgt(x, y):
    """
    Publishes the target coordinates.
    """
    global pub_target

    target = Point()

    target.x = x
    target.y = y
    target.z = 0

    pub_target.publish(target)

def get_info_goal(req):
    """
    Service callback function for 'goal_info' service.
    Returns the target reached and target canceled counts.
    """
    global target_reached, target_canceled, service

    return targetResponse(target_reached, target_canceled)

def set_target_params(x, y):
    """
    Sets the target coordinates as rosparams.
    """
    rospy.set_param('/target_x', x)
    rospy.set_param('/target_y', y)

def main():
    """
    Main function of node_a.
    - Initializes ROS node and necessary elements.
    - Sends goal requests to the server.
    - Handles user input for canceling the target.
    - Prints the target reached and target canceled counts.
    """
    global pub_info, target_reached, target_canceled, service, pub_target

    # Initialization of elements
    pose = PoseStamped()
    target_reached = 0
    target_canceled = 0

    # Init node
    rospy.init_node('node_a')

    # Create a new client
    client = actionlib.SimpleActionClient('/reaching_goal', assignment_2_2023.msg.PlanningAction)

    # Publish
    pub_info = rospy.Publisher('/bot_info', Info, queue_size=1)
    pub_target = rospy.Publisher('/tgt', Point, queue_size=1)

    # Subscribe to /odom
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    service = rospy.Service('goal_info', target, get_info_goal)

    # Wait for the server ready
    client.wait_for_server()

    # Get target coordinates using rosparam
    x_goal = rospy.get_param('/target_x', default=1.0)
    y_goal = rospy.get_param('/target_y', default=2.0)
    
    ltk_tgt(x_goal, y_goal)
    pose.pose.position.x = x_goal
    pose.pose.position.y = y_goal
    pose.pose.position.z = 0

    # User input for target coordinates
    while True:
        try:
            x_goal_user = float(input("Enter x-coordinate for the target: "))
            y_goal_user = float(input("Enter y-coordinate for the target: "))
            break
        except ValueError:
            print("Please enter valid numbers.")

    # Set target parameters using rosparam
    set_target_params(x_goal_user, y_goal_user)

    ltk_tgt(x_goal_user, y_goal_user)
    pose.pose.position.x = x_goal_user
    pose.pose.position.y = y_goal_user
    pose.pose.position.z = 0

    # Create the object PlanningGoal and assign the position goal
    goal = assignment_2_2023.msg.PlanningGoal(target_pose=pose)

    # Send the goal request
    client.send_goal(goal)

    # Wait for the goal to be reached
    client.wait_for_result()

    # Example: printing the final counts
    print("Target Reached: ", target_reached, flush=True)
    print("Target Canceled: ", target_canceled, flush=True)

if __name__ == "__main__":
    main()

