# Research Track 1, 2nd Assignment
The assignment is about the development of a package which has 3 nodes and interacts with a simulation of a simple robot in Gazebo. 
The nodes are:

- node a: 
 A node that implements an action client, allowing the user to set a target (x, y) or to cancel it. Try to use the feedback/status of the action server to know when the target has been reached. The node also publishesthe robot position and velocity as a custom message (x,y, vel_x, vel_z), by relying on the values published on the topic/odom.  

- node b:
 A service node that, when called, returnsthe coordinates of the last target sent by the user

- node c: 
 Another service node that subscribes to the robot's position and velocity (using the custom message) and implements a server to retrieve the distance of the robot from the target and the robot's average speed.


- Launch file
 There is also an implemented **.launch** file that starts the whole simulation, and also shows the nodes a and c on separate windows in the output.


# Installing and Running
The purpose of this assignment is to develop a ROS package with 3 ROS nodes that provide a way to interact with the environment of assignment_2_2023 package. The nodes are written with Python language, as well as the directory and the CMakeList file are modified. A launch file is also developed for executing the code.

To run the code, we need these steps : 

- A ROS Noetic
 We also open the terminal and in the bashrc file we write 
 ```python
 source /opt/ros/noetic/setup.bash
 ```

- Run the ROS core by executing this command in terminal:
```python
roscore

```
- Creating a ROS worksapace:
```python
mkdir -p catkin_ws/src
cd catkin_ws/
catkin_make
```
- Source the new setup.bash file in .bashrc : 
```python
source ~/catkin_ws/devel/setup.bash
```
Move to the src folder of the workspace:
```python
 cd catkin_ws/src   
```
- First we can lone the package assignment_2_2023 which provides an initial implementation of the robot environment : 
```python
git clone https://github.com/CarmineD8/assignment_2_2023
```
- Clone the package of my solution:
```python
git clone https://github.com/HodaMn/ResearchTrack1_Assignment2.git
```
- Again we use:
```python
cd ~/catkin_ws 
catkin_make
```
- Now, we can run the whole project by running the launch file:
```python
   roslaunch assignment_2_2023 assignment1.launch
```
- To call node b, write this in the root workspace folder:
```python
  rosrun assignment_2_2023 node_b.py
```


Nodes
---------

### node_a.py ###

The first py file creates a ROS node for robot interaction and enables the user to assign new target by getting the x and y coordinates. 

### node_b.py ###

The second py file creates a ROS node of a service to return the last desired position of a robot.

For showing the output of node b, write this code 

```python
rosrun assignment_2_2023 node_b.py
```
This will return the last desired x and y positions of the robot.

Generally you can write this line of code for nodes a and c to see their output in the terminal.

### node_c.py ###

The third py file creates a ROS node that provides a service to return the average velocity and the distance between the current and desired positions of a robot and updates the values of robot's position and velocity over time. 


Pseudocode for node a
----------

1. Import necessary libraries and modules

2. Define global variables for pose and twist

3. Define callback function for `/odom` topic:
    1. Extract position and velocity information from the message
    2. Create an Info message and publish it to `/bot_info` topic

4. Define function to publish target coordinates:
    1. Create a Point message with the given coordinates
    2. Publish the Point message to `/tgt` topic

5. Define service callback function for `goal_info` service:
    1. Return the counts of target reached and target canceled

6. Define function to set target coordinates as rosparams:
    1. Set the target coordinates as rosparams `/target_x` and `/target_y`

7. Define the main function:
    1. Initialize ROS node and necessary elements
    2. Create an action client for `/reaching_goal` action server
    3. Create publishers and subscribers for `/bot_info`, `/tgt`, and `/odom`
    4. Wait for the action server to be ready
    5. Get initial target coordinates using rosparam
    6. Publish initial target coordinates
    7. Prompt user for updated target coordinates
    8. Set updated target parameters using rosparam
    9. Publish updated target coordinates
    10. Create a `PlanningGoal` object with the updated target coordinates
    11. Send the goal request to the action server
    12. Wait for the goal to be reached
    13. Print the final state of target reached and target canceled

8. If the script is the main program, call the `main` function



