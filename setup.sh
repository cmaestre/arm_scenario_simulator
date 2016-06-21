#!/bin/sh

package_path=$(rospack find arm_scenario_simulator)

# Set GAZEBO env variables and 
export GAZEBO_MODEL_PATH=$package_path/models:$GAZEBO_MODEL_PATH
echo "GAZEBO_MODEL_PATH = "$GAZEBO_MODEL_PATH
