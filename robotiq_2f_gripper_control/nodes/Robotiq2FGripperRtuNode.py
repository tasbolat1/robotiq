#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Robotiq, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Robotiq, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2012, Robotiq, Inc.
# Revision $Id$

import sys
import time

import robotiq_2f_gripper_control.baseRobotiq2FGripper
import robotiq_modbus_rtu.comModbusRtu
import rospy
from robotiq_2f_gripper_control.msg._Robotiq2FGripper_robot_input import Robotiq2FGripper_robot_input
from robotiq_2f_gripper_control.msg._Robotiq2FGripper_robot_output import Robotiq2FGripper_robot_output
from sensor_msgs.msg import JointState

pub_topic = 'Robotiq2FGripperRobotInput'
sub_topic = 'Robotiq2FGripperRobotOutput'

timer_tuple = None


def main(device):
    global timer_tuple
    rospy.init_node('robotiq2FGripper')

    client = robotiq_modbus_rtu.comModbusRtu.communication(device)
    while not rospy.is_shutdown():
        if client.connect():
            break
        time.sleep(3.0)

    gripper = robotiq_2f_gripper_control.baseRobotiq2FGripper.robotiqbaseRobotiq2FGripper(client)

    def sub_callback(data):
        global timer_tuple
        timer_tuple = [data, time.time(), 0]
        gripper.refreshCommand(data)

    rospy.Subscriber(sub_topic, Robotiq2FGripper_robot_output, sub_callback, tcp_nodelay=True)
    pub = rospy.Publisher(pub_topic, Robotiq2FGripper_robot_input, queue_size=10, tcp_nodelay=True)
    #pub_js = rospy.Publisher("/joint_states", JointState, queue_size=10)

    rate = rospy.Rate(200.0)

    while not rospy.is_shutdown():
        status = gripper.sendAndGet()

        if timer_tuple is not None:
            timer_tuple[2] += 1

        if status is not None:
            if timer_tuple is not None and timer_tuple[0].rPR == status.gPR:
                t = (time.time() - timer_tuple[1]) * 1000
                print("Response in {:0.1f}ms / {} rw cycle(s)".format(t, timer_tuple[2]))
                timer_tuple = None

            pub.publish(status)
            js = gripper.get_joint_state()
            #pub_js.publish(js)
        else:
            rospy.logerr_throttle(5, "serial error.. please restart node")

        rate.sleep()


if __name__ == '__main__':
    try:
        main(sys.argv[1])
    except rospy.ROSInterruptException:
        pass
