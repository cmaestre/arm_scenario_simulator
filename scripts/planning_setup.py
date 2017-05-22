#!/usr/bin/env python

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
from std_msgs.msg import Empty


def axis_to_quat(axis, angle):
    sin = math.sin(angle/2)
    return Quaternion(w=math.cos(angle/2), x=sin*axis[0], y=sin*axis[1], z=sin*axis[2])

class Environment:

    def __init__(self):
        self.objects = {}
        self.rate = 100
        rospy.Subscriber('/environment/reset', Empty, self.reset_callback, queue_size = 1)

    def reset_callback(self, message):
        self.del_objects()
        self.init()

    def del_objects(self):
        self.objects['light_table0'].turn_off()
        self.objects['light_table1'].turn_off()
        self.objects['light_table2'].turn_off()        
        for obj in self.objects.keys(): 
            self.objects[obj].delete()        

    def add_object(self, obj):
        if obj: self.objects[obj.gazebo_name] = obj

    def init(self):
        ''' Create handles to (and spawn) simulated object in Gazebo'''        
        self.add_object( arm_sim.Lever('lever1').spawn( Point(x=0.5, y=0.6, z=0.76), orientation=axis_to_quat([0,0,1], math.pi/2) ) )
        self.add_object( arm_sim.Button('button1').spawn( Point(x=0.5, y=0.3, z=0.76) ) )
        self.add_object( arm_sim.Button('button2').spawn( Point(x=0.5, y=0, z=0.76) ) )

#        self.add_object( arm_sim.GazeboObject('cube1').spawn(    'DREAM_cube',      Point(x=0.5, y=0.55, z=0.78) ))
#        self.add_object( arm_sim.GazeboObject('cube2').spawn(    'DREAM_cube',      Point(x=0.4, y=0.2, z=0.78) ))
#        self.add_object( arm_sim.GazeboObject('cylinder1').spawn('DREAM_cylinder',  Point(x=0.75, y=0.05, z=0.79) ))
#        self.add_object( arm_sim.GazeboObject('cylinder2').spawn('DREAM_cylinder',  Point(x=0.45, y=0.35, z=0.79) ))

        # the following objects are not spawn, cause already present in the world before this script is run
        self.add_object( arm_sim.Pocket('table/pocket') )
        self.add_object( arm_sim.Light('light_table0', color=[0,0,255]) )
        self.add_object( arm_sim.Light('light_table1', color=[255,0,0]) )
        self.add_object( arm_sim.Light('light_table2', color=[0,255,0]) )

        # to let the publishers notify the master, so that the following commands are not discarded
        rospy.sleep(1)
        # Initialize objects attributes (color) and lights states
        self.objects['light_table0'].set_light_state(on = False, force = True) # force=True ensures sending a ros message to gazebo to set the gazebo_object's color, no matter what the current python object state is
        self.objects['light_table1'].set_light_state(on = False, force = True) # when force is False or ommited, the python object only publish a ros message if it considers it is necessary
        self.objects['light_table2'].set_light_state(on = False, force = True) # it is recommended to force at initialization time to ensure having a consistent state

        self.objects['lever1'].set_base_color(rgba = [0,0,255])
        self.objects['lever1'].set_lever_color(rgba = [0,0,255])

        self.objects['button1'].set_base_color(rgba = [255,0,0])
        self.objects['button1'].set_button_color(rgba = [255,0,0])
        
        self.objects['button2'].set_base_color(rgba = [0,255,0])
        self.objects['button2'].set_button_color(rgba = [0,255,0])        

#        self.objects['cube1'].set_color(rgba = [230,220,10])
#        self.objects['cube2'].set_color(rgba = [200,20,20])
#        self.objects['cylinder1'].set_color(rgba = [50,50,220])

        

    def run(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            try: self.rules()
            except KeyError: pass #When deleting models, key errors may occur
            rate.sleep()

    def rules(self):            
            objects = self.objects
    
            objects['light_table0'].set_light_state( objects['lever1'].is_pushed() )
            
            if not objects['lever1'].is_pushed():
                objects['light_table0'].turn_off()
                objects['light_table1'].turn_off()
                objects['light_table2'].turn_off()
            else:  
                if objects['button1'].is_pressed():
                    objects['light_table1'].turn_on() if not objects['light_table1'].is_on() else \
                    objects['light_table1'].turn_off()
                    time.sleep(1)
                    
                if objects['light_table1'].is_on() and objects['button2'].is_pressed():            
                    objects['light_table2'].turn_on() if not objects['light_table2'].is_on() else \
                    objects['light_table2'].turn_off()                
                    time.sleep(1)
                    
        # to access an objects pose : 
        # objects[name].get_state().pose


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
