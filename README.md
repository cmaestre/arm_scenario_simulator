##To install :
0. First make sure you have installed:
    1. ROS indigo (full desktop install, http://wiki.ros.org/indigo/Installation/Ubuntu)
    2. Baxter Simulator (http://sdk.rethinkrobotics.com/wiki/Simulator_Installation)
1. clone this repostiory into your catkin workspace "src" folder
2. execute ```sudo ./fix_gazebo_setup.sh``` (this tiny script edits two lines in a gazebo configuration file)


##To run :
1. Start by executing ```./baxter.sh sim``` while in the catkin_ws to be able to use Baxter
2. Move to the arm_scenario_simulator package (```roscd arm_scenario_simulator```) and source setup.sh : ```source setup.sh```
3. The scenario's environment can then be spawned executing ```roslaunch arm_scenario_simulator baxter_world.launch```