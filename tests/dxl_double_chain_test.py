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

    # checking that the set_all/get_all position commands work
    position_list = [[pi,pi,pi,pi], [1,1,2,2], [0,0,0,0], [pi,pi,pi,pi]]
    for pl in position_list:
        mm.set_all_command_position(pl)
        mm.wait(2)
        print mm.get_all_current_position()
        assert (np.allclose(mm.get_all_current_position(), pl, atol=.05))
