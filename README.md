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

![Screenshot from 2016-06-21 20:45:51.png](https://bitbucket.org/repo/GLdKKe/images/288501238-Screenshot%20from%202016-06-21%2020:45:51.png)
These topic respectively relay the state of button (1 if pressed, 0 else), the state of the lever (1 for one side, 0 for the other) and the presence of an object in the pocket (1 if yes, 0 if not).

In addition to the 3 original cameras coming with Baxter (head_camera, left and right hand_camera), a camera called "head_camera_2"has been added on Baxter's head, as the capture belows shows. This new camera, unlike the original head_camera, sees entirely the table.

![ABC](https://bitbucket.org/repo/GLdKKe/images/656878577-Screenshot%20from%202016-06-21%2020:50:38.png)

![Screenshot from 2016-06-21 20:58:43.png](https://bitbucket.org/repo/GLdKKe/images/3053352798-Screenshot%20from%202016-06-21%2020:58:43.png)

##Already done : 
* A table with a pocket and a sensor publishing (on a ROS topic) whether there is an object inside the pocket or not.
* Models for interactive objects (buttons, levers) with sensors publishing the state on objects on both Gazebo and ROS topics.
* Models for basic objects (cube and cylinder).
* Additional camera mounted on top of Baxter's head which points to the table (unlike Baxter's original head camera which stares too high).
* Provided by RethinkRobotics : simple ROS interface to control Baxter (in python).

##To do :
* Test if it is possible to increase simulation speed
* Improve textures of table and objects