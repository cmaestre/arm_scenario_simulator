#!/usr/bin/env python

from __future__ import print_function

import argparse
import sys
import os
import copy
import math
import random
import time

import rospy
import arm_scenario_simulator as arm_sim

from geometry_msgs.msg import Point, Quaternion
from arm_scenario_simulator.srv import *
#from std_msgs.msg import Empty
from gazebo_msgs.srv import GetModelState
import traceback


def axis_to_quat(axis, angle):
    sin = math.sin(angle/2)
    return Quaternion(w=math.cos(angle/2), x=sin*axis[0], y=sin*axis[1], z=sin*axis[2])

class Environment:

    def __init__(self):
        self.objects = {}
        self.rate = 100
#        rospy.Subscriber('/environment/reset', Empty, self.reset_callback, queue_size = 1)
        self.reset_service = rospy.Service('/env/restart_environment', 
                                            RestartWorld, 
                                            self.reset_callback)
        self.get_object_pose_service = rospy.Service('/env/get_object_state', 
                                            GetObjectState, 
                                            self.get_object_pose_callback)
    
    def get_object_pose_callback(self, req):
        obj_name = req.object_name
        try:
            client = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            obj_pose = client(obj_name, 'world')
            return GetObjectStateResponse([obj_pose.pose.position.x,
                                           obj_pose.pose.position.y,
                                           obj_pose.pose.position.z - 0.785])

        except AttributeError:
            traceback.print_exc()
        except:
            print('get_object_pose_callback - Error getting pose of object', obj_name)

    def reset_callback(self, req):
        if req.option == 'setup':
            self.del_objects()
            self.init()
        elif req.option == 'object':
            obj_name = req.model_name
            self.objects[obj_name].delete()
            req_x = req.model_pos_x
            req_y = req.model_pos_y
            req_z = req.model_pos_z
            if 'button' in obj_name:
                self.add_object( arm_sim.Button(obj_name).spawn( 
                    Point(x=req_x, y=req_y, z=req_z + 0.76) ) )
            elif 'lever' in obj_name:
                self.add_object( arm_sim.Lever(obj_name).spawn( 
                    Point(x=req_x, y=req_y, z=req_z + 0.76), orientation=axis_to_quat([0,0,1], math.pi/2) ) )
            elif 'cube' in obj_name:
                self.add_object( arm_sim.GazeboObject('cube1').spawn('DREAM_cube',
                    Point(x=req_x, y=req_y, z=req_z + 0.76) ))
            elif 'cylinder' in obj_name:
                self.add_object( arm_sim.GazeboObject('cylinder1').spawn('DREAM_cylinder',
                    Point(x=req_x, y=req_y, z=req_z + 0.76) ))
            else:
                print('reset_callback - Object name to spawn unknown:', obj_name)
                return False
        else:
            print('reset_callback - Option name unknown:', req.option)
            return False
        
        return True
        

    def del_objects(self):
        self.objects['light_table0'].turn_off()      
        for obj in self.objects.keys(): 
            self.objects[obj].delete()        

    def add_object(self, obj):
        if obj: self.objects[obj.gazebo_name] = obj

    def init(self):
        ''' Create handles to (and spawn) simulated object in Gazebo'''
        self.add_object( arm_sim.Button('button1').spawn( 
            Point(x=0.5, y=0.3, z=0.76) ) )
        self.add_object( arm_sim.GazeboObject('cube1').spawn(
            'DREAM_cube', Point(x=0.5, y=0.45, z=0.8) ))

        # the following objects are not spawn, cause already present in the world before this script is run
        self.add_object( arm_sim.Light('light_table0', color=[0,255,0]) )

        # to let the publishers notify the master, so that the following commands are not discarded
        rospy.sleep(1)
        # Initialize objects attributes (color) and lights states
        self.objects['light_table0'].set_light_state(on = False, force = True)
        self.objects['button1'].set_base_color(rgba = [0,255,0])
        self.objects['button1'].set_button_color(rgba = [0,255,0])
        
    def run(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            try: self.rules()
            except KeyError: pass #When deleting models, key errors may occur
            rate.sleep()

    def rules(self):            
        objects = self.objects
        
        print(objects['button1'].is_pressed(), objects['button1'].state_changed())

        if objects['button1'].is_pressed() and objects['button1'].state_changed():
            objects['light_table0'].turn_on() if not objects['light_table0'].is_on() else \
            objects['light_table0'].turn_off()
            time.sleep(5)

def main():
    rospy.init_node("DREAM_environment_demo")
    env = Environment()
    try:
        env.init()
        rospy.on_shutdown(env.del_objects)
        print("Running. Ctrl-c to quit")
        env.run()
    except Exception: 
        env.del_objects()
        raise


if __name__ == '__main__':
    try: main()
    except rospy.ROSInterruptException: pass
