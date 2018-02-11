from robot_player import MotionManager, DxlOptions
from time import sleep

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
    assert(mm.get_current_position([1,2,3,4]) == mm.get_all_current_position())
    mm.set_command_position([1,2], [1,1])
    mm.wait(2)

    mm.set_command_position([3,4], [-1,-1])
    mm.wait(2)

    mm.set_command_position([1,3],[0,0])
    mm.wait(2)

    mm.set_command_position([2,4], [0,0])
    mm.wait(2)
