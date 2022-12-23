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
#
# Modifed from the orginal comModbusTcp by Kelsey Hawkins @ Georgia Tech

from __future__ import division, print_function

from math import ceil

import pymodbus.version
import serial
from packaging import version
from pymodbus.client.sync import ModbusSerialClient
from pymodbus.exceptions import ModbusIOException

# version 3.5 has set_low_latency_mode
assert version.parse(serial.__version__) >= version.parse("3.5")
# version 1.3.X has timeout bug
assert version.parse(pymodbus.__version__) >= version.parse("2.0.0")


# sudo stty -F /dev/ttyUSB0 -echo -echoe -echok
# sudo stty -F /dev/ttyUSB0 raw

class communication:
    def __init__(self, port):
        self.port = port
        self.client = ModbusSerialClient(method='rtu', port=port, stopbits=1, bytesize=8, baudrate=115200, timeout=0.1)

    def connect(self):
        if not self.client.connect():
            print("[ERROR] Unable to connect to {}".format(self.port))
            return False

        try:
            self.client.socket.set_low_latency_mode(True)
            print("{} set_low_latency_mode ok".format(self.port))
        except Exception:
            print("please run: setserial {} low_latency".format(self.port))

        return True

    def disconnect(self):
        self.client.close()

    def write(self, data):
        if len(data) % 2 == 1:
            data.append(0)

        message = []
        for i in range(0, len(data) / 2):
            message.append((data[2 * i] << 8) + data[2 * i + 1])

        try:
            self.client.write_registers(0x03E8, message, unit=0x0009)
        except Exception as e:
            return False
        return True

    def read(self, num_bytes):
        num_regs = int(ceil(num_bytes / 2.0))

        try:
            response = self.client.read_holding_registers(0x07D0, num_regs, unit=0x0009)
        except Exception as e:
            return None

        if response is None:
            return None

        output = []
        for i in range(0, num_regs):
            output.append((response.getRegister(i) & 0xFF00) >> 8)
            output.append(response.getRegister(i) & 0x00FF)

        return output

    def readwrite(self, data, num_bytes):
        num_regs = int(ceil(num_bytes / 2.0))

        data_write = []
        for i in range(0, int(len(data) / 2)):
            data_write.append((data[2 * i] << 8) + data[2 * i + 1])

        try:
            rq = self.client.readwrite_registers(
                unit=0x0009,
                read_address=0x07D0, read_count=num_regs,
                write_address=0x03E8, write_registers=data_write,
            )
            if not isinstance(rq, ModbusIOException):
                data_read = []
                for i in range(0, num_regs):
                    data_read.append((rq.registers[i] & 0xFF00) >> 8)
                    data_read.append(rq.registers[i] & 0x00FF)
                return data_read
        except Exception as e:
            pass
        return None
