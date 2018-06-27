from robot_player import MotionManager, DxlOptions
from robot_player.dxl.dxl_control_table import XSERIES
from time import sleep
import platform
from math import pi
import numpy as np
"""
test for chain of X series dynamixels, 3 on a single chain plugged into each port.
Baudrate is 3000000
ID is [1,2,3]
"""

def allclose(l1, l2, tol=2.):
    for i, j in zip(l1, l2):
        if abs(i - j) > tol:
            raise AssertionError("elements of lists are {}, which is not within tolerance {}".format(abs(i - j), tol))
    return True

ids = [1,2,3]
motor_ids = [[1,2,3]]

if platform.system() == 'Windows':
    ports = ['COM15','COM16']
else:
    ports = ['/dev/ttyUSB0']

dopts = DxlOptions(motor_ids,
                   motor_types=['XSERIES'],
                   ports=ports,
                   baudrate=3000000,
                   protocol_version=2)

def test_set_command_position():
    with MotionManager(ids, dt=.005, options=dopts) as mm:

        # checking that the set_all/get_all position commands work
        position_list = [[pi,pi,pi], [1,2,3], [0,0,0], [pi,pi,pi]]
        for pl in position_list:
            mm.set_goal_position([1, 2, 3], pl)
            mm.wait(2.5)
            print(mm.get_all_present_position())

            get_curr_pos = mm.get_present_position([1, 2, 3 ])
            get_all_curr_pos = mm.get_all_present_position()
            assert(np.allclose(get_curr_pos, get_all_curr_pos, atol=.05))
            assert (np.allclose(mm.get_all_present_position(), pl, atol=.05))

def test_set_all_command_position():
    with MotionManager(ids, dt=.005, options=dopts) as mm:
        # checking that the set_all/get_all position commands work
        position_list = [[pi, pi, pi], [1, 2, 3], [0, 0, 0], [pi, pi, pi]]
        for pl in position_list:
            mm.set_all_goal_position(pl)
            mm.wait(2.5)
            print(mm.get_all_present_position())

            get_curr_pos = mm.get_present_position([1, 2, 3])
            get_all_curr_pos = mm.get_all_present_position()
            assert (np.allclose(get_curr_pos, get_all_curr_pos, atol=.05))
            assert (np.allclose(mm.get_all_present_position(), pl, atol=.05))

def test_velocity():
    with MotionManager(ids, dt=.005, options=dopts) as mm:
        mm.device.set_torque_enable(ids,[0]*3)
        mm.device._write_data(ids, XSERIES.OPERATING_MODE, [1]*3, 1  ) # set motors to be in velocity mode
        print(mm.device._read_data(ids, XSERIES.OPERATING_MODE, 1  )) # set motors to be in velocity mode
        mm.device.set_torque_enable(ids, [1]*3)

        mm.set_goal_velocity(ids, [pi / 2] * 3)  # test positive velocity
        sleep(4)
        allclose(mm.get_present_velocity(ids), [pi / 2] * 3, .1)
        mm.set_goal_velocity(ids, [0] * 3)

        mm.set_goal_velocity(ids, [-pi / 2] * 3)  # test negative velocity
        sleep(4)
        mm.set_goal_velocity(ids, [0] * 3)
        allclose(mm.get_present_velocity(ids), [-pi / 2] * 3, .1)

        mm.set_goal_velocity(ids, [pi, pi / 2, pi / 3])  # test different velocities
        sleep(4)
        allclose(mm.get_present_velocity(ids), [pi, pi / 2, pi / 3], .1)
        mm.set_goal_velocity(ids, [0] * 3)

        sleep(1)
        allclose(mm.get_present_velocity(ids), [0] * 3, .1)  # test zero velocity

        mm.device.set_torque_enable(ids, [0]*3)
        mm.device._write_data(ids, XSERIES.OPERATING_MODE, [3]*3, 1)  # set motors to be in position mode
        print(mm.device._read_data(ids, XSERIES.OPERATING_MODE, 1))  # check that motors are in position mode
if __name__ == '__main__':
    test_set_all_command_position()
    test_set_command_position()
    test_velocity()