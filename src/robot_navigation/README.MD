# Robot Navigation Configuration

This package consist of all of the navigation configuration of the robot. Navigation configuration refers to the documentation of ros navigation package in the [ros 1 wiki](https://wiki.ros.org/navigation/Tutorials/RobotSetup) below
![ROS Navigation Stack Setup](https://wiki.ros.org/navigation/Tutorials/RobotSetup?action=AttachFile&do=get&target=overview_tf_small.png)

Prerequisites:
1. AMCL Package
2. Map server Package
3. move_base package

Navigation stack setup for amcl, map_server, and move_base are handled in this package. The remaining platform specific node is described in the rest of the package
The remaining package configures:
1. Sensor transforms: robot_description
2. Odometry source: robot_odometry
3. Sensor sources: robot_bringup
4. Base controller: robot_bringup

Internal configuration of the move_base package is located in this package's config directory. Configuration consist of:
1. amcl : amcl_params.yaml; This configuration mainly didn't have to be touched if you have the same RPLiDAR A1M8, A2M8, etc.
2. common costmap (global and local) params: common_costmap_params.yaml; Detailed information of each parameter's effect on the local costmap is documented [here](https://wiki.ros.org/navigation/Tutorials/RobotSetup#Costmap_Configuration_.28local_costmap.29_.26_.28global_costmap.29). You have to consider to optimize it to your platform, especially for: footprint, inflation radius, obstacle range, and raytrace range
3. global costmap : global_costmap_params.yaml; This param usually didn't have to be changed too.
4. local costmap : local_costmap_params.yaml; This param usually didn't have to be changed too.
5. local planner : dwa_local_planner_params.yaml; THIS HAVE TO BE CONFIGURED TO YOUR OWN PLATFORM. You can check its configuration on your own local planner, however in this package it uses dynamic window approach package. You may edit it to use another local planner however you like. To use another local planner, you can read more [here](http://wiki.ros.org/move_base) (how to edit it in move_base launch file(especially in base_local_planner params)) and [here](https://wiki.ros.org/nav_core) (options of local planners)
6. global planner : in this project, it doesn't have to be specified because it uses the default global planner. To edit it, you can change the base_global_planner assuming you have installed the global planner name and its class is already installed correctly as in local planner above

Steps on how to create your own platform:
1. Create your own hardware. There are three types of mobile robot steering (as far as i know): ackermann, differential drive, and holonomic.
To create your own hardware, you have to first know how they're different to each other. Second, you have to equip your hardware to sensors to quantify its parameters (e.g., speed, steer angle, etc).
2. Tune your hardware. If you use this project, then you have to use [this repo](https://github.com/samiframadhan/firwmare_robot) and follow the PID Tuning guidance there. Other than that, you may use ros_controller package and tune it accordingly.
3. Create your robot's URDF. Measure each sensor's distance and pose according to your robot's centric point (x:0, y:0, z:0), and write down each sensor link and joints. You can find example in the robot_description package
4. If you use this project's repo, then you have already set the odometry translation from your hardware sensor data in robot_odometry package. Otherwise, you have to create your own or use ros_controller package
5. Tune the odometry. You may have the perfect measurement of your robot's dimension, but you have to make sure your robot's odometry is accurate enough. You can check it by running the odometry translation (in this project case, it's robot_odometry) alongside your robot node package (in this project case, it's robot_bringup's bringup.launch) launch file. And then try to make a designated rectangular path. If it's not looking like one, you may troubleshoot it or you can adjust its node parameters.
6. If you are confident that your odometry is great already, you may proceed to start trying localize your robot according to your map. However, most DIY robots isn't that accurate so i suggest you to use some form of filtering algorithm to smooth out your odometry and compare it to another sensor. In this project case, i am using robot_localization package that is implemented in robot_ekf package in this repo. Try it once, you may find your robot moves a bit wiggly or not moving at all at the first time but trust me that if it's configured correctly it would have your robot's movement filtered nicely.
7. Do a SLAM!. To use move_base package, you are assumed to already have your own environment map. Of course you may didn't have one, so you can try to make one by running SLAM algorithm using gmapping, hector slam, or google cartographer. The latter needs additional configuration, but it is worth mentioning since i started from that. To run it, you must have run the odometry nicely and you can use robot_slam package launch file(TODO)
8. Save your map to the maps directory, and change the map.launch's file name to your map name. Look the [map_server documentation](http://wiki.ros.org/map_server) on how to do that
9. Try to localize your robot using AMCL's package. It will find where is your robot according to your saved map. To run AMCL, you have to run: bringup, tf_transforms, odometry, map_server, and amcl
10. If the localization success, you will be able to run the navigation launch (navigation_with_rviz.launch) smoothly.

There you have it folks.