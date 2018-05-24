#!usr/bin/env python
__author__ = "Joshua Hooks, Daniel Sun"
__email__ = "danielsun@ucla.edu"
__copyright__ = "Copyright 2018 RoMeLa"
__date__ = "May 16, 2018"

__version__ = "0.0.1"
__status__ = "Prototype"

import mscl_imu.mscl as mscl
from mscl_imu.mscl import Error as MSCL_Error
import numpy as np
import collections

"""
Sets up a connection to the 3DM-GX4-25 Microstrain IMU
"""

IMUData = collections.namedtuple('IMUData','new_data euler_angles base_ang_vel lin_acc ')

class imu_interface(object):

    def __init__(self, sample_rate, com_port="/dev/ttyACM0"):
        # Expected data euler angles and angular rates
        self.data = {'estRoll': 0.0, 'estPitch': 0.0, 'estYaw': 0.0,
                     'estAngularRateX': 0.0, 'estAngularRateY': 0.0, 'estAngularRateZ': 0.0,
                     'estLinearAccelX': 0.0, 'estLinearAccelY': 0.0, 'estLinearAccelZ': 0.0}

        # try:
        self.com_port = com_port
        self.connection = mscl.Connection.Serial(self.com_port, 921600)
        self.node = mscl.InertialNode(self.connection)

        chs_filter = mscl.MipChannels()
        chs_filter.append(mscl.MipChannel(mscl.MipTypes.CH_FIELD_ESTFILTER_ESTIMATED_ORIENT_EULER, mscl.SampleRate.Hertz(sample_rate)))
        chs_filter.append(mscl.MipChannel(mscl.MipTypes.CH_FIELD_ESTFILTER_ESTIMATED_ANGULAR_RATE, mscl.SampleRate.Hertz(sample_rate)))
        chs_filter.append(mscl.MipChannel(mscl.MipTypes.CH_FIELD_ESTFILTER_ESTIMATED_LINEAR_ACCEL, mscl.SampleRate.Hertz(sample_rate)))

        self.node.setActiveChannelFields(mscl.MipTypes.CLASS_ESTFILTER, chs_filter)
        self.node.enableDataStream(mscl.MipTypes.CLASS_ESTFILTER)
        self.node.resume()
        # except mscl.Error as e:
        #     print("Error:", e)

    def __enter__(self):
        return self

    def __exit__(self, exception_type, exception_value, traceback):
        self._close()

    def __del__(self):
        self._close()

    def _close(self):
        self.node.setToIdle()
        self.connection.disconnect()


    def read(self):
        new_data = self.update()
        euler = [self.data['estRoll'], self.data['estPitch'], self.data['estYaw']]
        ang_rate = [self.data['estAngularRateX'], self.data['estAngularRateY'], self.data['estAngularRateZ']]
        lin_acc = [self.data['estLinearAccelX'], self.data['estLinearAccelY'], self.data['estLinearAccelZ']]
        out = IMUData(new_data=new_data, euler_angles=euler, base_ang_vel=ang_rate, lin_acc=lin_acc)
        return out

    def update(self):
        packets = self.node.getDataPackets(10)
        n_packets = packets.size()
        new_data = False
        if n_packets > 0:
            new_data = True
            packet = packets[n_packets-1]
            points = packet.data()
            for dataPoint in points:
                self.data[dataPoint.channelName()] = dataPoint.as_float()
        return new_data

    # def __del__(self):

