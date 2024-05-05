#!/usr/bin/env python3
import sys
import rospy
import actionlib
import threading
import time
import actionlib.msg
import assignment_2_2023.msg
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Twist, PoseStamped
from assignment_2_2023.msg import PlanningAction, PlanningGoal
from assignment_2_2023.msg import Info
from assignment_2_2023.srv import target, targetResponse

# Global Variables
target_reached = 0
target_canceled = 0
current_goal = None
stop_thread = None

# Odometry callback function to publish robot information
def clbk_odom(msg):
    """
    Callback function for '/odom' topic.
    Updates the position and velocity information.
    """
    pub_info.publish(Info(
        x=msg.pose.pose.position.x,
        y=msg.pose.pose.position.y,
        vel_x=msg.twist.twist.linear.x,
        vel_y=msg.twist.twist.linear.y
    ))

# Service callback to get information about goals
def get_info_goal(req):
    """
    Service callback function for 'goal_info' service.
    Returns the target reached and target canceled counts.
    """
    return targetResponse(target_reached, target_canceled)


# Function to handle user input for stopping or exiting
def check_for_stop(client):
    global target_canceled
    while not rospy.is_shutdown():
        command = input("Enter a command (type 'stop' to cancel the current goal or 'exit' to quit): ")
        if command.lower() == "stop":
            client.cancel_goal()
            target_canceled += 1
            print("Goal canceled.")
            break
        elif command.lower() == "exit":
            rospy.signal_shutdown("Exiting program")
            break

# Main function to handle ROS node initialization and goal management
def main():
    """
    Main function of node_a.
    - Initializes ROS node and necessary elements.
    - Sends goal requests to the server.
    - Handles user input for canceling the target.
    - Prints the target reached and target canceled counts.
    """
    
    global pub_info, current_goal, target_reached, target_canceled, stop_thread

    # Initialize ROS node
    rospy.init_node("node_a")

    # Set up publishers, subscribers, and services
    pub_info = rospy.Publisher("/bot_info", Info, queue_size=1)
    rospy.Subscriber("/odom", Odometry, clbk_odom)
    rospy.Service("goal_info", target, get_info_goal)

    # Create Action Client and wait for the Action Server
    client = actionlib.SimpleActionClient("/reaching_goal", PlanningAction)
    client.wait_for_server()

    while not rospy.is_shutdown():
        # User input for goal coordinates
        try:
            x_goal = float(input("Enter x-coordinate for the target: "))
            y_goal = float(input("Enter y-coordinate for the target: "))
        except ValueError:
            print("Invalid input, please enter numerical values.")
            continue

        # Create and send the goal
        pose = PoseStamped()
        pose.pose.position.x = x_goal
        pose.pose.position.y = y_goal
        pose.pose.position.z = 0
        goal = PlanningGoal(target_pose=pose)

        client.send_goal(goal)
        current_goal = goal

        # Start or restart the stop thread
        if stop_thread is None or not stop_thread.is_alive():
            stop_thread = threading.Thread(target=check_for_stop, args=(client,))
            stop_thread.start()

        # Wait for the goal result
        client.wait_for_result()

        # Check if the goal was achieved or canceled
        if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            target_reached += 1
            print("Goal reached!")
        elif client.get_state() == actionlib.GoalStatus.PREEMPTED:
            print("Goal preempted.")
        elif client.get_state() == actionlib.GoalStatus.ABORTED:
            print("Goal aborted.")
        
        print(f"Targets Reached: {target_reached}, Targets Canceled: {target_canceled}")

if __name__ == "__main__":
    main()
