#!/usr/bin/env python

import argparse
import struct
import sys
import os
import copy
import math

import rospy
import rospkg
rospack = rospkg.RosPack()

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import (
    Header,
    Empty,
)

import baxter_interface

def axis_to_quat(axis, angle):
    sin = math.sin(angle/2)
    return Quaternion(w=math.cos(angle/2), x=sin*axis[0], y=sin*axis[1], z=sin*axis[2])

gazebo_models_path = rospack.get_path('arm_scenario_simulator')+'/models'

objects_to_load = []
objects_to_load.append({'name' : 'table',
                        'path' : gazebo_models_path+'/DREAM_setup/model.sdf',
                        'pose' : Pose(position=Point(x=0.6, y=0.0, z=0.0), orientation=axis_to_quat([0,0,1], -math.pi/2 )) })

objects_to_load.append({'name' : 'button1',
                        'path' : gazebo_models_path+'/push_button/model.sdf',
                        'pose' : Pose(position=Point(x=0.6, y=-0.1, z=0.76)) })

objects_to_load.append({'name' : 'lever1',
                        'path' : gazebo_models_path+'/lever/model.sdf',
                        'pose' : Pose(position=Point(x=0.8, y=0.5, z=0.76), orientation=axis_to_quat([0,0,1], -math.pi/2 )) })

objects_to_load.append({'name' : 'cube1',
                        'path' : gazebo_models_path+'/cube/model.sdf',
                        'pose' : Pose(position=Point(x=0.7, y=0.3, z=0.76)) })

objects_to_load.append({'name' : 'cylinder1',
                        'path' : gazebo_models_path+'/cylinder/model.sdf',
                        'pose' : Pose(position=Point(x=0.6, y=0.2, z=0.76)) })
sucessfully_loaded = []


def run():
    rospy.spin()

def load_gazebo_models():
    for obj in objects_to_load:
        xml = ''
        with open (obj['path'], "r") as f:
            xml=f.read().replace('\n', '')
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        try:
            spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
            resp_sdf = spawn_sdf(obj['name'], xml, "/", obj['pose'], "world")
            sucessfully_loaded.append(obj)
        except rospy.ServiceException, e:
            rospy.logerr("Spawn SDF service call failed: {0}".format(e))


def delete_gazebo_models():
    # This will be called on ROS Exit, deleting Gazebo models
    # Do not wait for the Gazebo Delete Model service, since
    # Gazebo should already be running. If the service is not
    # available since Gazebo has been killed, it is fine to error out
    for obj in sucessfully_loaded:
        try:
            delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
            resp_delete = delete_model(obj['name'])
        except rospy.ServiceException, e:
            rospy.loginfo("Delete Model service call failed: {0}".format(e))


def main():
    rospy.init_node("ik_pick_and_place_demo")
    # Load Gazebo Models via Spawning Services
    # Note that the models reference is the /world frame
    # and the IK operates with respect to the /base frame
    load_gazebo_models()
    # Remove models from the scene on shutdown
    rospy.on_shutdown(delete_gazebo_models)
    rospy.wait_for_message("/robot/sim/started", Empty)
    run()


if __name__ == '__main__':
    sys.exit(main())
