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
import rospy
from robotiq_2f_gripper_control.msg._Robotiq2FGripper_robot_input import Robotiq2FGripper_robot_input
from robotiq_2f_gripper_control.msg._Robotiq2FGripper_robot_output import Robotiq2FGripper_robot_output
from sensor_msgs.msg import JointState


def clamp(val, _min, _max):
    assert _max >= _min
    return min(max(val, _min), _max)


def clamp_command(command):
    # type: (Robotiq2FGripper_robot_output) ->  Robotiq2FGripper_robot_output
    command.rACT = clamp(command.rACT, 0, 1)
    command.rGTO = clamp(command.rGTO, 0, 1)
    command.rATR = clamp(command.rATR, 0, 1)
    command.rPR = clamp(command.rPR, 0, 255)
    command.rSP = clamp(command.rSP, 0, 255)
    command.rFR = clamp(command.rFR, 0, 255)
    return command


class robotiqbaseRobotiq2FGripper(object):
    def __init__(self, client):
        self.message = []
        self.client = client
        self.seq = 0
        self.status = None

    def refreshCommand(self, command):
        # type: (Robotiq2FGripper_robot_output) ->  None
        command = clamp_command(command)
        self.message = []
        self.message.append(command.rACT + (command.rGTO << 3) + (command.rATR << 4))
        self.message.append(0)
        self.message.append(0)
        self.message.append(command.rPR)
        self.message.append(command.rSP)
        self.message.append(command.rFR)

    def sendCommand(self):
        self.client.write(self.message)

    def getStatus(self):
        # type: () -> Robotiq2FGripper_robot_input
        status = self.client.read(6)

        message = Robotiq2FGripper_robot_input()
        message.gACT = (status[0] >> 0) & 0x01
        message.gGTO = (status[0] >> 3) & 0x01
        message.gSTA = (status[0] >> 4) & 0x03
        message.gOBJ = (status[0] >> 6) & 0x03
        message.gFLT = status[2]
        message.gPR = status[3]
        message.gPO = status[4]
        message.gCU = status[5]

        return message

    def sendAndGet(self):
        # type: () -> Robotiq2FGripper_robot_input
        """ Read and write at the same time ... could be faster """
        data_read = self.client.readwrite(self.message, 6)

        if data_read is not None:
            message = Robotiq2FGripper_robot_input()
            message.gACT = (data_read[0] >> 0) & 0x01
            message.gGTO = (data_read[0] >> 3) & 0x01
            message.gSTA = (data_read[0] >> 4) & 0x03
            message.gOBJ = (data_read[0] >> 6) & 0x03
            message.gFLT = data_read[2]
            message.gPR = data_read[3]
            message.gPO = data_read[4]
            message.gCU = data_read[5]
            self.status = message
            return message
        return None

    def get_joint_state(self):
        self.seq += 1
        js = JointState()
        js.header.frame_id = ''
        js.header.stamp = rospy.get_rostime()
        js.header.seq = self.seq
        if self.status is None:
            return js

        js.name = ['finger_joint']
        # 0.7 is the close position for 0.140m gripper
        js.position = [clamp((self.status.gPO - 3.0) / (230.0 - 3.0) * 0.7, 0, 0.7)]
        js.velocity = [0]
        js.effort = [0]
        return js
