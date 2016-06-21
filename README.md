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

At this point, if you run ```rostopic list```, you can observe many topics related to Baxter and three topics related to the environment listed here : 

![topics1.png](https://bitbucket.org/repo/GLdKKe/images/1099375156-topics1.png)

These topics respectively relay the state of the button (1 if pressed, 0 else), the state of the lever (1 for one side, 0 for the other) and the presence of an object in the pocket (1 if yes, 0 if not).

In addition to the 3 original cameras coming with Baxter (head_camera, left and right hand_camera), a camera called "head_camera_2"has been added on Baxter's head, as the capture belows shows. This new camera, unlike the original head_camera, entirely captures the table.

![topics2.png](https://bitbucket.org/repo/GLdKKe/images/655402798-topics2.png)

![head_camera_2.png](https://bitbucket.org/repo/GLdKKe/images/451840738-head_camera_2.png)
An snapshot of head_camera_2 shown in Rviz. A gaussian noise has been added to the image for more realism.


##Already done : 
* A table with a pocket and a sensor publishing (on a ROS topic) whether there is an object inside the pocket or not.
* Models for interactive objects (buttons, levers) with sensors publishing the state on objects on both Gazebo and ROS topics.
* Models for basic objects (cube and cylinder).
* Additional camera mounted on top of Baxter's head which points to the table (unlike Baxter's original head camera which stares too high).
* Provided by RethinkRobotics : simple ROS interface to control Baxter (in python).

##To do :
* Test if it is possible to increase simulation speed
* Improve textures of table and objects