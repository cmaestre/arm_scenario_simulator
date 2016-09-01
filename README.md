# Introduction

This package was made to allow people make developmental robotic experiment simulations in Gazebo, with Baxter or any other robots having an arm (necessary to interact with the world).
It includes the models and python wrappers for several Gazebo objects, some of them are interactive or contain sensors.

This allows to create simple environments where a robot can perform actions (pushing a button, putting something in a pocket, pushing a lever) having different consequences (turning on a light, moving an object).
This way one can test algorithms for policy learning, curiosity, ...

The package comes with an example demonstrating Baxter.

# How to

## Install :
0. First make sure you have installed:
    1. ROS indigo (full desktop install, http://wiki.ros.org/indigo/Installation/Ubuntu)
    2. Baxter Simulator (http://sdk.rethinkrobotics.com/wiki/Simulator_Installation)
1. clone this repostiory into your catkin workspace "src" folder
2. In the catkin_ws folder, run ```catkin_make```.


## Run :
1. Start by executing ```./baxter.sh sim``` while in the catkin_ws to be able to use Baxter
2. Move to the arm_scenario_simulator package (```roscd arm_scenario_simulator```) and source setup.sh : ```source setup.sh```
3. The example environment can then be laucnhed by executing ```roslaunch arm_scenario_simulator baxter_world.launch``` 
4. Then some objects can be spwaned on the table by executing ```rosrun arm_scenario_simulator spawn_objects_example``` resulting in something like this :

![gazebo.png](https://bitbucket.org/repo/GLdKKe/images/874311045-gazebo.png)

### Basic setup
Launching the world `baxter_world.launch` will start a ros master, gazebo and spawn:

- baxter robot with all its original sensors and actuators (cameras, grippers, lasers ...) (defined in this package, mainly copied from baxter_gazebo package)
- a table with borders and a pocket with a presence sensor (defined in this package)
- three lights fixed to the table (defined in this package)

*Note :* In addition to the 3 original cameras coming with Baxter (head_camera, left and right hand_camera), a camera called "head_camera_2"has been added on Baxter's head, as the capture belows shows. This new camera, unlike the original head_camera, entirely captures the table.

![topics2.png](https://bitbucket.org/repo/GLdKKe/images/655402798-topics2.png)

![pov.png](https://bitbucket.org/repo/GLdKKe/images/2147259962-pov.png)

An snapshot of head_camera_2 shown. A gaussian noise has been added to the image for more realism.

### Additional objects

In addition to this basic setup, the arm_scenario_simulator comes with other objects:

- buttons
- levers
- lights
- cubes and cylinders

All these objects can be spawned, moved and deleted directly from python code, allowing you to automate experiments initialization : no need to write one `.world` file for each experiment setup you want, just write a python script to spawn a variable amount of objects at different positions on the table or to reset the setup, etc ...

The state of buttons, levers and the table's pocket can also be retrieved easily from python code, and the lights can be switched on/off directly from python as well. This enables you to define simple or complex rules relating objects states and lights (e.g, turn on light2 iif this buttonC in pressed, more examples in the file linked few lines below).

You can set the color of all these objects, and get/set their physical state (position and velocity) in python too.

All this is made possible using the python classes contained in the python [arm_scenario_simulator package](https://bitbucket.org/u2isir/arm_scenario_simulator/src/8d92c844061e778f5237e0dc58fe971463a7594d/src/arm_scenario_simulator/?at=master).

The example script [spawn_objects_example](https://bitbucket.org/u2isir/arm_scenario_simulator/src/1685739a91dc1a0840ca1bd89dc1bc6fcdefdd0f/scripts/spawn_objects_example?at=master&fileviewer=file-view-default) aims at demonstrating how to do this.
**This serves as a tuto to demonstrate how this package works/can be used**

**Important note : color changes made at run-time with python or c++ code are only visible through cameras (therefore using rviz or image_view package), not in gazebo's window client.**


## Control Baxter
Please refer to rethink Robotics' [examples](https://github.com/RethinkRobotics/baxter_examples) and [python api for baxter's arms](http://api.rethinkrobotics.com/baxter_interface/html/index.html)

# Road map

## Already done : 
* A table with a pocket and a sensor publishing (on a ROS topic) whether there is an object inside the pocket or not.
* Models for interactive objects (buttons, levers) with sensors publishing the state on objects on both Gazebo and ROS topics.
* Models for basic objects (cube and cylinder).
* Python interface for these objects allowing controlling them and/or retrieving their state.
* Additional camera mounted on top of Baxter's head which points to the table (unlike Baxter's original head camera which stares too high).
* Provided by RethinkRobotics : simple ROS interface to control Baxter (in python).

## To do :
* Improve textures of table

# About simulating faster than real time with Gazebo :

To my knowledge, it is possible to speed up the simulation of the physics engine by editing the file worlds/setup.world in two ways :

1. increasing ```real_time_update_rate``` (usually 1'000), which is the number of update steps performed (per real second) to evolve the world.
2. increasing ```max_step_size``` (usually 0.001), which is the virtual length (the duration in simulated world) of the update step mentioned at the previous point.

```real_time_update_rate``` times per second, the physics engine makes the simulated world progress by ```max_step_size``` second, therefore the simulated world goes ```max_step_size * real_time_update_rate``` times faster than the real world.

* By increasing ```real_time_update_rate``` while keeping ```max_step_size``` constant, you increase the required computational power of the simulation.
* By increasing ```max_step_size``` while keeping ```real_time_update_rate``` constant, -I think- you decrease the simulation quality (linearizations are less accurate)

Increasing the physics engine speeds do not speed up sensors: if you increase physics engine speed, the apparent update rate of sensors in the simulated world will be decreased, since the update rates defined in .sdf files are expressed w.r.t real time. Make sure your increase the sensors update rate if you want to keep the same apparent update rate.

On my computer, the limiting factor seems to be the cameras. Indeed they seem to be not able to capture more than 30 frames per (real) second : increasing the update_rate in models/DREAM_baxter/urdf/baxter_base/baxter_base.gazebo.xacro over 30 does not make the rate of the topic /cameras/*_camera/image go higher. Therefore, after speeding up the physics engine by a factor 2, the apparent update rate of the camera is a little less than 15 - which is still correct. However, it seems that this maximum frame rate is divided by the number of open cameras. So if a second camera is recording, each camera should have frame rate of 15 at physics speed x1, and 7.5 at physics speed x2. 

Other sensors like buttons, levers and presence in table's pocket have a much higher (real) update rate (in fact it is ```real_time_update_rate```) so even if their apparent update rate is decrease when accelerating the physics, this should not be the main issue compared to the cameras.