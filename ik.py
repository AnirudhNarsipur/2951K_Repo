#!/usr/bin/env python


import argparse
import struct
import sys

import rospy

import rospy

import math
import tf2_ros

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

rospy.init_node("rsdk_ik_service_client")
def ik_test(limb,lpos,lor,rpos,ror):
    
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    poses = {
        'left': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=lpos["x"],
                    y=lpos["y"],
                    z=lpos["z"],
                ),
                orientation=Quaternion(
                    x=lor["x"],
                    y=lor["y"],
                    z=lor["z"],
                    w=lor["w"]
                ),
            ),
        ),
        'right': PoseStamped(
            header=hdr,
            pose=Pose(
                position=Point(
                    x=rpos["x"],
                    y=rpos["y"],
                    z=rpos["z"],
                ),
                orientation=Quaternion(
                    x=ror["x"],
                    y=ror["y"],
                    z=ror["z"],
                    w=ror["w"]
                ),
            ),
        ),
    }

    ikreq.pose_stamp.append(poses[limb])
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1

    # Check if result valid, and type of seed ultimately used to get solution
    # convert rospy's string representation of uint8[]'s to int's
    resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                               resp.result_type)
    if (resp_seeds[0] != resp.RESULT_INVALID):
        seed_str = {
                    ikreq.SEED_USER: 'User Provided Seed',
                    ikreq.SEED_CURRENT: 'Current Joint Angles',
                    ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                   }.get(resp_seeds[0], 'None')
        print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
              (seed_str,))
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        print "\nIK Joint Solution:\n", limb_joints
        print "------------------"
        print "Response Message:\n", resp
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")

    return 0

def get_transform(tag_name):

    # rospy.init_node('tf2_april_listener')

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(10.0)
    try:
        trans = tfBuffer.lookup_transform("base", tag_name, rospy.Time())
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException), e:
        print("Error! " , e)
        return None
    pos = {
        "x" : trans.translation.x , 
        "y" : trans.translation.y , 
        "z" : trans.translation.z ,   

    }
    ornt = {
        "x" : trans.translation.x , 
        "y" : trans.translation.y , 
        "z" : trans.translation.z ,   
        "w" : trans.translation.w 

    }
    return pos,ornt 

def main():
    """RSDK Inverse Kinematics Example
    A simple example of using the Rethink Inverse Kinematics
    Service which returns the joint angles and validity for
    a requested Cartesian Pose.
    Run this example, passing the *limb* to test, and the
    example will call the Service with a sample Cartesian
    Pose, pre-defined in the example code, printing the
    response of whether a valid joint solution was found,
    and if so, the corresponding joint angles.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    parser.add_argument(
        '-l', '--limb', choices=['left', 'right'], required=True,
        help="the limb to test"
    )
    args = parser.parse_args(rospy.myargv()[1:])

    lpos , lor = get_transform("tag_0")

    return ik_test(args.limb,lpos,lor,lpos,lor)

if __name__ == '__main__':
    sys.exit(main())