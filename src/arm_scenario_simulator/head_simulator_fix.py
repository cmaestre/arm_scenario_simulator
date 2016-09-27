'''
This scripts exists because baxter's gazebo simulator seems to not support the speed argument given to head.set_pan.
Indeed, the head turns almost instantenously to the pan command given to set_pan, indepenently to the speed command given.
'''
import os
import subprocess

import rospy
from baxter_interface import Head
from baxter_core_msgs.msg import HeadPanCommand

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
                    rospy.logerr('Could not reach the target angle (+/- '+str(margin)+' rad) within '+str(timeout)+ 'sec')
                    break
                rate.sleep()
    Head.move_to_pan = move_to_pan #attach a new method to Head class


    try: # look if the node already exists
        rospy.wait_for_service('/head_controller_simulator_fix', 5.0)
    except:
        path = os.path.join(os.path.dirname(__file__), 'head_controller_simulator_fix')
        head_controler_fix_node = subprocess.Popen(['python', path])
        def clean_up():
            head_controler_fix_node.kill()
        rospy.on_shutdown(clean_up)
