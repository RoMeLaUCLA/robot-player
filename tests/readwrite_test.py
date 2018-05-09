from robot_player import MotionManager, DxlOptions
from time import sleep
from math import pi
import numpy as np
"""
test for chains of MX106 dynamixels, two plugged into each port.
"""

ids = [1,2,3,4]
motor_ids = [(1,2),(3,4)]
dopts = DxlOptions(motor_ids,
                   motor_types=['MX106','MX106'],
                   ports=['/dev/ttyUSB0','/dev/ttyUSB1'],
                   baudrate=3000000,
                   protocol_version=2)

with MotionManager(ids, dt=.005, options=dopts) as mm:
    # mm.initialize()

    di = mm.device
    di.set_goal_position([1,2,3,4], [1,1,1,1])
    address = 7
    print di._read_data([1,2,3,4], address, 1)

    address = 11
    print di._read_data([1,2,3,4], address, 1)
    # print di._read_data([1,2,3,4], 132, 4)
    # print "operating mode: " + str(di._read_data([1], 11, 1))
    # print "read function: " + str(di._read_data([1], 132, 4))
    # should return a list

