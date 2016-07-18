#!/usr/bin/env python

import math
import random
import argparse

import rospy

from tf import transformations as tft
np = tft.numpy

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import (Header, String, Empty)
from sensor_msgs.msg import JointState

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

import baxter_interface
from baxter_interface import CHECK_VERSION

def IK(limb, position, orientation):
    ns = "ExternalTools/" + limb.name + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    pose = PoseStamped(
            header=Header(stamp=rospy.Time.now(), frame_id='base'),
            pose=Pose(position,orientation)
        )
    names = ['left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2']
    positions = [-1.5352305689404009, 1.4906375729922043, -0.038481535830354296, 0.1940629459492209, 1.5458392657611721, 1.4966324394341486, -0.5197399373727922]
    velocities, efforts = [0]*7, [0]*7
    seed = JointState(Header(stamp=rospy.Time.now(), frame_id='base'), names, positions, velocities, efforts)

    ikreq.pose_stamp.append(pose)
    ikreq.seed_angles.append(seed)
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        raise Exception

    if (resp.isValid[0]):
        print("SUCCESS - Valid Joint Solution Found :")
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        print "\nIK Joint Solution:\n", limb_joints
        print "------------------"
        print "Response Message:\n", resp
        return limb_joints
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")
        raise Exception

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    #parser.add_argument('side', dest='side', required=True, choices=['left','right'], help="side to use to babble")
    args = parser.parse_args(rospy.myargv()[1:])

    print("Initializing babbler node... ")
    rospy.init_node("node_clement")
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()
    print("Running. Ctrl-c to quit")
    x0, y0 = 0.6, 0.3
    d_x, d_y = 0.3, 0.4
    d_a = 20
    limb = baxter_interface.Limb('left')
    down = True

    for k in range(30):
        z = -0.15 if down else 0
        x = x0 + random.uniform(-d_x,d_x)
        y = y0 + random.uniform(-d_y,d_y)

        angle = (math.pi/180)*(180 + random.uniform(-d_a,d_a))
        vec = tft.unit_vector([1, random.uniform(-0.1,0.1), random.uniform(-0.1,0.1)] )
        orientation=Quaternion(
            x=math.sin(angle/2)*vec[0],
            y=math.sin(angle/2)*vec[1],
            z=math.sin(angle/2)*vec[2],
            w=math.cos(angle/2),
        )
        try:
            joints = IK(limb, Point(x,y,z), orientation)
            limb.move_to_joint_positions(joints, timeout = 5)
            down = not down
        except Exception as e:pass
