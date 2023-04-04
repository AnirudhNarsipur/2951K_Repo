#!/usr/bin/env python

# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Baxter RSDK Joint Position Example: keyboard
"""
import argparse

import rospy

import baxter_interface
import baxter_external_devices

from baxter_interface import CHECK_VERSION


def map_keyboard():
    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    lj = left.joint_names()
    rj = right.joint_names()

    def vector_diff(v1,v2):
        return sum([abs(i -j) for i,j in zip(v1.values(),v2.values())])

    def set_j(limb, joint_name, delta):
        current_position = limb.joint_angle(joint_name)
        joint_command = {joint_name: current_position + delta}
        limb.set_joint_positions(joint_command)
       
    
    
    def set_arm_pos(limb,pos_dict):
        limb.set_joint_positions(pos_dict)
        diff =vector_diff(limb.joint_angles(),pos_dict)
        while diff > 0.001:
            limb.set_joint_positions(pos_dict)
            print("vector diff is ",diff)
            print(limb.joint_angles())
            y=raw_input("break?")
            if y=="y":
                break
            pass


    default_l_pos = {
        lj[0] : 0.00383 ,
        lj[1] : 1.285150,
        lj[2] : -0.014189,
        lj[3] : -0.00920388,
        lj[4] : 0.742446,
        lj[5] : -0.00383,
        lj[6] : -0.55146609,

    }
    default_r_pos = {
        rj[0] : 0.00766,
        rj[1] : -0.55338,
        rj[2] : 0.006135,
        rj[3] : 1.25748,
        rj[4] : 0.0130388,
        rj[5] : 0.0084368,
        rj[6] : 0.744747,

    }
    mv_l_pos = {
        lj[0] : 0.097024,
        lj[1] : 1.27090,
        lj[2] : 0.087820,
        lj[3] : -0.025310,
        lj[4] : 0.5957102,
        lj[5] : -0.92384,
        lj[6] : -0.33402,

    }
    mv_r_pos = {
        rj[0] : 0.85826,
        rj[1] : -0.465561,
        rj[2] : 0.00766699,
        rj[3] : 0.9641069,
        rj[4] : 0.06519418,
        rj[5] : -0.0928058,
        rj[6] : 0.886257,

    }
    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    print("left join_states",left.joint_angles())
    
    print("right joint states ",right.joint_angles(),"\n")
    set_arm_pos(left,default_l_pos)
    set_arm_pos(right,default_r_pos)
    set_arm_pos(left,mv_l_pos)
    set_arm_pos(right,mv_r_pos)
    set_arm_pos(left,default_l_pos)
    set_arm_pos(right,default_r_pos)
    rospy.signal_shutdown("Example finished.")


def main():
    """RSDK Joint Position Example: Keyboard Control

    Use your dev machine's keyboard to control joint positions.

    Each key corresponds to increasing or decreasing the angle
    of a joint on one of Baxter's arms. Each arm is represented
    by one side of the keyboard and inner/outer key pairings
    on each row for each joint.
    """
    epilog = """
See help inside the example with the '?' key for key bindings.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__,
                                     epilog=epilog)
    parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("rsdk_joint_position_keyboard")
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    def clean_shutdown():
        print("\nExiting example...")
        if not init_state:
            print("Disabling robot...")
            rs.disable()
    rospy.on_shutdown(clean_shutdown)

    print("Enabling robot... ")
    rs.enable()

    map_keyboard()
    print("Done.")


if __name__ == '__main__':
    main()