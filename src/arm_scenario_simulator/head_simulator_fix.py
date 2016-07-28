'''
This scripts exists because baxter's gazebo simulator seems to not support the speed argument given to head.set_pan.
Indeed, th head turns almost instantenously to the pan command given to set_pan, indepenently to the speed command given.
'''

import rospy
import roslaunch
from baxter_interface import Head

from baxter_core_msgs.msg import (
   HeadPanCommand,
   HeadState,
)

try:
    rospy.wait_for_service('/cameras/list', timeout=1) # test if the service is available
    rospy.loginfo("You're using the robot")
except rospy.ROSException: # if not, it is probably cause we're using the simulator
    rospy.loginfo("You are using the simulator")

    fix_publisher = rospy.Publisher('/robot/head/command_head_pan_simulator_fix',  HeadPanCommand, queue_size=10)
    def set_pan_speed(self, angle, speed):
        msg = HeadPanCommand(angle, speed, True)
        fix_publisher.publish(msg)


    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    head_controler_fix_node = launch.launch(roslaunch.core.Node('arm_scenario_simulator', 'head_controler_fix'))

    def clean():
        head_controler_fix_node.close()
        launch.stop()
    rospy.on_shutdown(clean)

    Head.set_pan_speed = set_pan_speed #attach a new method to Head class
