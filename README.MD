# What this repository is about

This repo contains a complete ROS Navigation Stack excluding firmware of an example robot which seperated to other [repo](https://github.com/samiframadhan/firmware_robot).

To use this repo, one should run the install script
```bash
chmod +x ./install_external_packages.sh
./install_external_packages.sh
```

Then, assuming the robot's hardware have been perfected, you could run the autonomous mode by sourcing the workspace
```bash
cd <path to ros1_ws>

catkin_make

source devel/setup.bash

roslaunch robot_navigation robot_navigation.launch
```

By default, the configuration of the mobile robot uses the default navigation stack of ROS.
To find out the detailed configuration, you may proceed to see the robot_navigation package README file

## TODO: Verify SLAM, Localization, Navigation