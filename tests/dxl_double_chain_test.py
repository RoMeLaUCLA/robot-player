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

def test_set_command_position():
    with MotionManager(ids, dt=.005, options=dopts) as mm:

        # checking that the set_all/get_all position commands work
        position_list = [[pi,pi,pi,pi], [1,1,2,2], [0,0,0,0], [pi,pi,pi,pi]]
        for pl in position_list:
            mm.set_goal_position([1, 2, 3, 4], pl)
            mm.wait(2.5)
            print(mm.get_all_present_position())

            get_curr_pos = mm.get_present_position([1, 2]) + mm.get_present_position([3, 4])
            get_all_curr_pos = mm.get_all_present_position()
            assert(np.allclose(get_curr_pos, get_all_curr_pos, atol=.05))
            assert (np.allclose(mm.get_all_present_position(), pl, atol=.05))

def test_set_all_command_position():
    with MotionManager(ids, dt=.005, options=dopts) as mm:
        # checking that the set_all/get_all position commands work
        position_list = [[pi, pi, pi, pi], [1, 1, 2, 2], [0, 0, 0, 0], [pi, pi, pi, pi]]
        for pl in position_list:
            mm.set_all_goal_position(pl)
            mm.wait(2.5)
            print(mm.get_all_present_position())

            get_curr_pos = mm.get_present_position([1, 2]) + mm.get_present_position([3, 4])
            get_all_curr_pos = mm.get_all_present_position()
            assert (np.allclose(get_curr_pos, get_all_curr_pos, atol=.05))
            assert (np.allclose(mm.get_all_present_position(), pl, atol=.05))

if __name__ == '__main__':
    test_set_all_command_position()
    test_set_command_position()