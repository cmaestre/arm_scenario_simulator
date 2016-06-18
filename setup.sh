#!/bin/sh

package_path=$(rospack find arm_scenario_simulator)

# Gather the package's plugin libraries into a unique lib directory
mkdir -p $package_path/lib
rm -f $package_path/lib/*.so
ln -f $package_path/models/*/plugins/build/*.so $package_path/lib/
echo "Found plugin libraries :"
ls $package_path"/lib"

# Set GAZEBO env variables and 
export GAZEBO_MODEL_PATH=$package_path/models:$GAZEBO_MODEL_PATH
export GAZEBO_PLUGIN_PATH=$package_path/lib:$GAZEBO_PLUGIN_PATH

echo "GAZEBO_MODEL_PATH = "$GAZEBO_MODEL_PATH
echo "GAZEBO_PLUGIN_PATH = "$GAZEBO_PLUGIN_PATH
