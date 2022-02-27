# Dan's baby ROS cheatsheet!

### Access the docker container of ros:noetic:
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