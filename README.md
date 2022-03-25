# ros_motor_controller_driver
A motor controller driver created in cpp, and wrapped using ROS:Noetic!

## How to use:
* The 'robot_driver' package is to be placed in the 'src' directory of your catkin workspace.

## Descrption:
* I wanted to become more familiar with ROS, so I decided to make a ROS wrapper for a cpp motor controller driver!

## Implementation:
Here are some addition things I included for simplicity / coolness:
* I added a launch file such that parameters would not have to be repetitively initilized
* I added an Asynchronous Spinner to avoid delayed callback issues
* ### Access the docker container of ros:noetic:
* 'docker exec -it festive_shirley bin/bash'

### We want to cd into our catkin_ws and source our setup file:
* This is so that ros can find our package and we can run ros and catkin commands
* 'source devel/setup.bash'

### If any changes were made, remember to:
* 'catkin_make', 'source devel/setup.bash'

### To start the parameter server, in a new terminal:
* 'roscore'

### To start the driver, in a new terminal:
* 'roslaunch robot_driver rosparam.launch'

### To set speed via publishing to speed_command subscriber
* 'rostopic pub -1 /testRobot/speed_command std_msgs/Int32 "data: 10"'

### To stop the motor:
* 'rosservice call /testRobot/stop_motor "{}"'

# Models
### Starting the Parameter Server
![Screen Shot 2022-03-25 at 4 09 17 PM](https://user-images.githubusercontent.com/89366190/160194769-54a68877-0da7-4742-88b9-2671146d3047.png)

### Starting the Driver
![Screen Shot 2022-03-25 at 4 09 32 PM](https://user-images.githubusercontent.com/89366190/160194773-0d3e46ad-7a54-44ce-9311-513cb5e9b14f.png)

### Viewing Initial Speed
![Screen Shot 2022-03-25 at 4 11 32 PM](https://user-images.githubusercontent.com/89366190/160194778-dfc81883-c619-4a2d-a3b5-e1b65a7e908a.png)

### Viewing Motor Status
![Screen Shot 2022-03-25 at 4 12 01 PM](https://user-images.githubusercontent.com/89366190/160194785-26798ee8-c041-4345-adba-a1d5575a9133.png)

### Setting Motor Speed to 5
![Screen Shot 2022-03-25 at 4 13 44 PM](https://user-images.githubusercontent.com/89366190/160194790-4e503ef2-dd67-49d7-8b65-1eee8cc28254.png)

### Setting Motor Speed to 200 keeps it at 5 since Max Speed is 5
![Screen Shot 2022-03-25 at 4 14 18 PM](https://user-images.githubusercontent.com/89366190/160194794-e8d35031-f070-43d0-a930-c851d91866e6.png)

### Stopping the Motor
![Screen Shot 2022-03-25 at 4 14 37 PM](https://user-images.githubusercontent.com/89366190/160194799-dc08ca65-ba33-4d81-ad25-553dfc68ab59.png)
