from robot_player import MotionManager, DxlOptions
import numpy as np
import platform


"""
Test a single chain of DXLs 
IDs: 1,2
USB port is /dev/ttyUSB0
Motors are MX106s
"""

try:
    input = raw_input
except NameError:
    pass
motor_id = [1,2]
dt = .005

if platform.system() == 'Windows':
    ports = ['COM15']
else:
    ports = ['/dev/ttyUSB0']

dxl_num = int(input("Enter a number to choose dynamixel type. 1 for MX-106, 2 for DXLPRO"))
if dxl_num == 1:
    dxl_str = "MX106"
elif dxl_num == 2:
    dxl_str = "DXLPRO"
else:
    print("Please a number between 1 and 2.")
    exit(1)

dxl_opts = DxlOptions([motor_id],
            motor_types=[dxl_str],
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