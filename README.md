##To install :
0. First make sure you have installed:
    1. ROS indigo (full desktop install, http://wiki.ros.org/indigo/Installation/Ubuntu)
    2. Baxter Simulator (http://sdk.rethinkrobotics.com/wiki/Simulator_Installation)
1. clone this repostiory into your catkin workspace "src" folder
2. execute ```sudo ./fix_gazebo_setup.sh``` (this tiny script edits two lines in a gazebo configuration file)


##To run :
1. Start by executing ```./baxter.sh sim``` while in the catkin_ws to be able to use Baxter
2. Move to the arm_scenario_simulator package (```roscd arm_scenario_simulator```) and source setup.sh : ```source setup.sh```
3. The scenario's environment can then be laucnhed by executing ```roslaunch arm_scenario_simulator baxter_world.launch``` and then some objects can be spwaned on the table by executing ```rosrun arm_scenario_simulator spawn objects.py``` resulting in something like this :

![aperçu.png](https://bitbucket.org/repo/GLdKKe/images/3521778972-aper%C3%A7u.png)

##Already present : 
* A table with a pocket
* Models for interactive objects (buttons, levers) with sensors (a plugin publishing the state on a GAZEBO topic)
* Models for basic objects (cube and cylinder)
* provided by rethinkRobotics : simple ROS interface to control Baxter (in python)

##To Do :
* add a sensor to the table's pocket
* add textures to objects
* interfacing gazebo's sensor topic with ROS topics such that the state of buttons and lever is available in ROS
* testing simulation speed increase possibilities