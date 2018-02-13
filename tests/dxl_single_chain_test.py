from robot_player import MotionManager, DxlOptions
import numpy as np
from time import sleep

"""
Test a single chain of DXLs 
IDs: 1,2
USB port is /dev/ttyUSB0
Motors are MX106s
"""

motor_id = [1,2]
dt = .005
dxl_opts = DxlOptions([[1,2]],
            motor_types=['MX106'],
            ports=['/dev/ttyUSB0'],
            baudrate=3000000,
            protocol_version=2
           )

with MotionManager(motor_id, dt=dt, options=dxl_opts) as mm:
    # move and check
    mm.set_all_command_position([1,1])
    mm.wait(3)
    assert (np.allclose(mm.get_all_current_position(), np.array([1, 1]), atol=.1))
    print mm.get_all_current_position()

    # move back
    mm.set_all_command_position([0,0])
    mm.wait(3)
    mm.get_all_current_position()
    assert(np.allclose(mm.get_all_current_position(),np.array([0,0]),atol=.1))
    print mm.get_all_current_position()