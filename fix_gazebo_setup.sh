#!/bin/sh

# Edit gazebo's setup so that the following env variable changes are not reset at Gazebo's launch
gazebo_setup_path=$(pkg-config --variable=prefix gazebo)/share/gazebo/setup.sh
python fix_gazebo_setup.py $gazebo_setup_path
