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
    def move_to_pan(self, angle, speed, blocking = False, margin=0.03, timeout=5):
        start = rospy.get_time()
        msg = HeadPanCommand(angle, speed, True)
        fix_publisher.publish(msg)
        if blocking:
            rate = rospy.Rate(10)
            while abs(self.pan() - angle)>margin:
                if (rospy.get_time()-start)>timeout:
                    rospy.logerr('Could not reach the target angle (+/- '+margin+' rad) within '+str(timeout)+ 'sec')
                    break
                rate.sleep()
    Head.move_to_pan = move_to_pan #attach a new method to Head class


    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    head_controler_fix_node = launch.launch(roslaunch.core.Node('arm_scenario_simulator', 'head_controller_fix'))

    def clean_up():
        head_controler_fix_node.stop()
        launch.stop()
    rospy.on_shutdown(clean_up)
