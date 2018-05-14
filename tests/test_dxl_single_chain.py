from robot_player import MotionManager, DxlOptions
import numpy as np
import platform
from time import sleep

"""
Test a single chain of DXLs 
IDs: 1,2
USB port is /dev/ttyUSB0
Motors are MX106s
"""


motor_id = [1,2]
dt = .005

if platform.system() == 'Windows':
    ports = ['COM15']
else:
    ports = ['/dev/ttyUSB0']

dxl_opts = DxlOptions([[1,2]],
            motor_types=['MX106'],
            ports=ports,
            baudrate=3000000,
            protocol_version=2
           )

with MotionManager(motor_id, dt=dt, options=dxl_opts) as mm:
    # move and check
    mm.set_all_goal_position([1, 1])
    mm.wait(3)
    assert (np.allclose(mm.get_all_present_position(), np.array([1, 1]), atol=.1))
    print(mm.get_all_present_position())

    # move back
    mm.set_all_goal_position([0, 0])
    mm.wait(3)
    mm.get_all_present_position()
    assert(np.allclose(mm.get_all_present_position(), np.array([0, 0]), atol=.1))
    print(mm.get_all_present_position())