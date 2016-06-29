#!/usr/bin/env python
import sys
import rospy

import baxter_interface
from baxter_interface import Limb, Head, RobotEnable, CHECK_VERSION
from std_msgs.msg import Empty


def main():
    head = Head()
    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    lj = left.joint_names()
    rj = right.joint_names()

    names = ['head_pan', 'l_gripper_l_finger_joint', 'l_gripper_r_finger_joint', 'left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2', 'r_gripper_l_finger_joint', 'r_gripper_r_finger_joint', 'right_e0', 'right_e1', 'right_s0', 'right_s1', 'right_w0', 'right_w1', 'right_w2']
    positions = [4.7021362090227115e-05, 0.020833001577423894, -0.020802621909745838, -1.55941638279986, 1.4830573354502796, 0.30013493161079285, -0.03461681012600959, 1.4532331295076775, 1.4978508263392714, -0.5200008135037466, 0.02083302541593118, 3.082853698827555e-08, 1.189972011707093, 1.9400294784400458, -1.2679671988152226, -0.9999845802509117, -0.6699967118262613, 1.0300087421376407, 0.49999968920096194]
    positions_dico = {names[i]:positions[i] for i in range(len(names))}

    left.move_to_joint_positions({joint:positions_dico[joint] for joint in lj})
    right.move_to_joint_positions({joint:positions_dico[joint] for joint in rj})
    grip_left.open()
    head.set_pan(0)


if __name__ == '__main__':
    try:
        rospy.init_node("goto_initpose")
        print("waiting for simulator ready... ")
        rospy.wait_for_message("/robot/sim/started", Empty)
        print("Enabling robot... ")
        rs = RobotEnable(CHECK_VERSION)
        rs.enable()
        print('Moving to initial pose ...')
        sys.exit( main() )
        print('Done')
    except rospy.ROSInterruptException,e :
        pass
