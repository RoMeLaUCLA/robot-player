"""
Test a single F/T sensor from the Dynamixels that's connected to motors with ids 5 and 6.

"""

from robot_player import MotionManager, DxlOptions, DxlInterface

motor_ids = [5,6]
dt = .01
dxl_options = DxlOptions(motor_ids=[motor_ids],
                     motor_types=['DXL_PRO'],
                     ports=['/dev/ttyUSB0'],
                     baudrate=3000000, protocol_version=2)

# DI = DxlInterface(dxl_options.baudrate, dxl_ports=dxl_options.dxl_ports)
#
# DI.initialize()

with MotionManager(motor_ids, dt, options=dxl_options) as mm:
    mm.initialize()
