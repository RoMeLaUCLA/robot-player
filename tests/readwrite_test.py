from robot_player import MotionManager, DxlOptions
from time import sleep
from math import pi
import numpy as np
"""
test for chains of MX106 dynamixels, two plugged into each port.
"""

def allclose(l1, l2, tol=2):
    for i,j in zip(l1, l2):
        if abs(i-j) > tol:
            raise AssertionError("elements of lists are {}, which is not within tolerance {}".format(abs(i-j), tol))
    return True

ids = [1,2,3,4]
motor_ids = [(1,2),(3,4)]
dopts = DxlOptions(motor_ids,
                   motor_types=['MX106','MX106'],
                   ports=['/dev/ttyUSB0','/dev/ttyUSB1'],
                   baudrate=3000000,
                   protocol_version=2)

with MotionManager(ids, dt=.005, options=dopts) as mm:
    di = mm.device
    ##  read data

    address = 7 # motor id
    assert([1,2,3,4] ==  di._read_data(ids, address, 1))

    # check to make sure that data gets read back in whatever order the ids were passed in.
    assert([3,4,2,1] ==  di._read_data([3,4,2,1], address, 1))


    address = 0 # model number
    assert(di._read_data(ids, address, 2) == [321,321,321,321])

    address = 20 # homing offset
    assert(di._read_data(ids, address, 4) == [0,0,0,0])


    ## write data

    # turn the LEDs on for 3 seconds and then off
    address = 65
    di._write_data(ids, address, [1]*len(ids), 1)
    sleep(3)
    di._write_data(ids, address, [0]*len(ids), 1)

    # torque enable off
    address = 64
    di._write_data(ids, address, [0] * len(ids), 1)

    # set the motors to be in velocity mode
    address = 11
    di._write_data(ids, address, [1]*len(ids), 1)
    assert ([1,1,1,1] == di._read_data(ids, address, 1))

    # torque enable on
    address = 64
    di._write_data(ids, address, [1] * len(ids), 1)

    # velocity of 100 for 3 seconds
    address = 104
    di._write_data(ids, address, [50]*len(ids), 4)
    sleep(3)
    di._write_data(ids, address, [0]*len(ids), 4)

    # torque enable off
    address = 64
    di._write_data(ids, address, [0] * len(ids), 1)

    # back to position mode
    address = 11
    di._write_data(ids, address, [3]*len(ids), 1)
    assert ([3,3,3,3] == di._read_data(ids, address, 1))

    # torque enable on
    address = 64
    di._write_data(ids, address, [1] * len(ids), 1)

    # set goal position to be 1000
    address = 116
    di._write_data([3,4,2,1], address, [1300, 1400, 1200, 1100], 4)
    sleep(3)

    # check that data is close
    pos_data = di._read_data([3,4,2,1], address, 4)
    allclose(pos_data, [1300, 1400, 1200, 1100], tol=2)

    # set goal position to be 0
    address = 116
    di._write_data(ids, address, [0,0,0,0], 4)
    sleep(3)

    # check that present position is back to zero
    address = 132
    pos_data = di._read_data(ids, address, 4)
    allclose(pos_data,[0,0,0,0],tol=2)



