#!/bin/sh

package_path=$(rospack find arm_scenario_simulator)

# Gather the package's plugin libraries into a unique lib directory
mkdir -p $package_path/lib
rm -f $package_path/lib/*.so
ln -f $package_path/models/*/plugins/build/*.so $package_path/lib/
echo "Found plugin libraries :"
ls $package_path"/lib"

# Edit gazebo's setup so that the following env variable changes are not reset at Gazebo's launch
gazebo_setup_path=$(pkg-config --variable=prefix gazebo)/share/gazebo/setup.sh
python fix_gazebo_setup $gazebo_setup_path

# Set GAZEBO env variables and 
export GAZEBO_MODEL_PATH=$package_path/models:$GAZEBO_MODEL_PATH
export GAZEBO_PLUGIN_PATH=$package_path/lib:$GAZEBO_PLUGIN_PATH

