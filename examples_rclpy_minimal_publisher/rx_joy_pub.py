#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.



from time import sleep

import rclpy

from std_msgs.msg import String
from sensor_msgs.msg import Joy

import asyncio
import serial
import serial_asyncio


# We do not recommend this style as ROS 2 provides timers for this purpose,
# and it is recommended that all nodes call a variation of spin.
# This example is only included for completeness because it is similar to examples in ROS 1.
# For periodic publication please see the other examples using timers.
class SBUSReceiver:
    class SBUSFramer(asyncio.Protocol):

        START_BYTE = 0x0f
        END_BYTE = 0x00
        SBUS_FRAME_LEN = 25

        def __init__(self):
            super().__init__()
            self._in_frame = False
            self.transport = None
            self._frame = bytearray()
            self.frames = asyncio.Queue()

        def connection_made(self, transport):
            self.transport = transport

        def data_received(self, data):
            for b in data:
                if self._in_frame:
                    self._frame.append(b)
                    if len(self._frame) == SBUSReceiver.SBUSFramer.SBUS_FRAME_LEN:
                        decoded_frame = SBUSReceiver.SBUSFrame(self._frame)
                        # print(decoded_frame)
                        asyncio.run_coroutine_threadsafe(self.frames.put(decoded_frame), asyncio.get_running_loop())
                        self._in_frame = False
                else:
                    if b == SBUSReceiver.SBUSFramer.START_BYTE:
                        self._in_frame = True
                        self._frame.clear()
                        self._frame.append(b)

        def connection_lost(self, exc):
            asyncio.get_event_loop().stop()

    class SBUSFrame:
        OUT_OF_SYNC_THD = 10
        SBUS_NUM_CHANNELS = 16
        SBUS_SIGNAL_OK = 0
        SBUS_SIGNAL_LOST = 1
        SBUS_SIGNAL_FAILSAFE = 2

        def __init__(self, frame):
            self.sbusChannels = [None] * SBUSReceiver.SBUSFrame.SBUS_NUM_CHANNELS

            channel_sum = int.from_bytes(frame[1:23], byteorder="little")

            for ch in range(0, 16):
                self.sbusChannels[ch] = channel_sum & 0x7ff
                channel_sum = channel_sum >> 11

            # Failsafe
            self.failSafeStatus = SBUSReceiver.SBUSFrame.SBUS_SIGNAL_OK
            if (frame[SBUSReceiver.SBUSFramer.SBUS_FRAME_LEN - 2]) & (1 << 2):
                self.failSafeStatus = SBUSReceiver.SBUSFrame.SBUS_SIGNAL_LOST
            if (frame[SBUSReceiver.SBUSFramer.SBUS_FRAME_LEN - 2]) & (1 << 3):
                self.failSafeStatus = SBUSReceiver.SBUSFrame.SBUS_SIGNAL_FAILSAFE

        def get_rx_channels(self):
            """
            Used to retrieve the last SBUS channels values reading
            :return:  an array of 18 unsigned short elements containing 16 standard channel values + 2 digitals (ch 17 and 18)
            """

            return self.sbusChannels

        def get_rx_channel(self, num_ch):
            """
            Used to retrieve the last SBUS channel value reading for a specific channel
            :param: num_ch: the channel which to retrieve the value for
            :return:  a short value containing
            """

            return self.sbusChannels[num_ch]

        def get_failsafe_status(self):
            """
            Used to retrieve the last FAILSAFE status
            :return:  a short value containing
            """

            return self.failSafeStatus

        def __repr__(self):
            return ",".join(str(ch) for ch in self.sbusChannels)

    def __init__(self):
        self._transport = None
        self._protocol = None

    @staticmethod
    async def create(port='/dev/ttyUSB2'):
        receiver = SBUSReceiver()
        receiver._transport, receiver._protocol = await serial_asyncio.create_serial_connection(
            asyncio.get_running_loop(),
            SBUSReceiver.SBUSFramer,
            port,
            baudrate=100000,
            parity=serial.PARITY_EVEN,
            stopbits=serial.STOPBITS_TWO,
            bytesize=serial.EIGHTBITS)
        return receiver

    async def get_frame(self):
        return await self._protocol.frames.get()

async def main(args=None):
#async def main():
    args = None
    rclpy.init(args=args)

    node = rclpy.create_node('joy')
    sbus = await SBUSReceiver.create("/dev/ttyUSB2")

    publisher = node.create_publisher(Joy, 'joy', 10)

    print('start')
    #msg = Joy()
    msg = Joy()

    i = 0
    while rclpy.ok():

        frame = await sbus.get_frame()    
        print(frame)
        data1 = (frame.sbusChannels[0]-1024)/1024
        data2 = (frame.sbusChannels[1]-1024)/1024
        data3 = (frame.sbusChannels[2]-1024)/1024
        data4 = (frame.sbusChannels[3]-1024)/1024
        data5 = (frame.sbusChannels[4]-1024)/1024
        data6 = (frame.sbusChannels[5]-1024)/1024
        msg.axes = [float(data1),float(data2),float(data3),float(data4),float(data5),float(data6)]
        print(msg)
        publisher.publish(msg)

    node.destroy_node()
    rclpy.shutdown()

loop = asyncio.get_event_loop()
loop.run_until_complete(main())
loop.run_forever()
loop.close()


