# What is this package about

This package contains launch files to start the robot's hardware. There are at least three hardwares; Motor control board, Lidar, and Joystick. 

Some motor control board specific constants can be configured from its yaml file located in the config directory in this package. However, to actually implement the parameter, one very likely should restart the ESP through the provided ros topic to do it ('/reset/esp') since the configuration takes place at the start of the sequence of board power up
Motor control board contains IMU which data is used for smoothen the change of orientation in z axis through robot_localization package. Before using the IMU, make sure the orientation of it is similar to what you have had in your  URDF file. The test can be done using test_imu.launch file.

Lidar used in this project is RPLiDAR A2M8, if your lidar is different you should consult on your LiDAR manufacturer on how to get the thing to work in ROS. Expected topic published is '/scan' and the message would be sensor_msgs/LaserScan

Joystick is an optional hardware, it used to control the robot from... joystick.