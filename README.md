##To install :
0. First make sure you have installed:
    1. ROS indigo (full desktop install, http://wiki.ros.org/indigo/Installation/Ubuntu)
    2. Baxter Simulator (http://sdk.rethinkrobotics.com/wiki/Simulator_Installation)
1. clone this repostiory into your catkin workspace "src" folder
2. In the catkin_ws folder, run ```catkin_make```.


##To run :
1. Start by executing ```./baxter.sh sim``` while in the catkin_ws to be able to use Baxter
2. Move to the arm_scenario_simulator package (```roscd arm_scenario_simulator```) and source setup.sh : ```source setup.sh```
3. The scenario's environment can then be laucnhed by executing ```roslaunch arm_scenario_simulator baxter_world.launch``` and then some objects can be spwaned on the table by executing ```rosrun arm_scenario_simulator spawn objects.py``` resulting in something like this :

![aperçu.png](https://bitbucket.org/repo/GLdKKe/images/3521778972-aper%C3%A7u.png)

##Already done : 
* A table with a pocket.
* Models for interactive objects (buttons, levers) with sensors publishing the state on objects on both Gazebo and ROS topics.
* Models for basic objects (cube and cylinder).
* Addtitional camera mounted on top of Baxter's head which points to the table (unlike Baxter's original head camera which stares too high).
* Provided by rethinkRobotics : simple ROS interface to control Baxter (in python).

##To do :
* add a sensor to the table's pocket
* add textures to objects
* testing simulation speed increase possibilities