#!/bin/sh

sudo apt update
sudo apt upgrade

sudo apt install -y ros-noetic-robot-localization \
    ros-noetic-slam-gmapping \
    ros-noetic-hector-slam \
    ros-noetic-navigation \
    ros-noetic-rosserial-arduino \
    ros-noetic-map-server

# optional
sudo apt install -y ros-noetic-joy