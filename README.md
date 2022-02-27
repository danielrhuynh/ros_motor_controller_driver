# ros_motor_controller_driver
A motor controller driver created in cpp, and wrapped using ROS:Noetic!

## How to use:
* The 'robot_driver' package is to be placed in the 'src' directory of your catkin workspace.

## Background:
* I wanted to become more familiar with ROS, so I decided to make a ROS wrapper for a cpp motor controller driver!

## Implementation:
* Here are some addition things I included for simplicity / coolness:
* I added a launch file such that parameters would not have to be repetitively initilized
* I added an Asynchronous Spinner to avoid delayed callback issues
