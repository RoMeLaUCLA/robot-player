from robot_player import MotionManager, DxlOptions
import numpy as np
import platform
import time

if platform.system() == 'Windows':
    ports = ['COM15', 'COM16']
else:
    ports = ['/dev/ttyUSB0']

motor_id = [2]
dt = .005

dxl_opts = DxlOptions([motor_id],
            motor_types=['DXLPRO'],
            ports=ports,
            baudrate=3000000,
            protocol_version=2
           )

with MotionManager(motor_id, dt=dt, options=dxl_opts) as mm:
    #Test effort_limit function, printing out the max effort in dxl and then N*m units

    print('The max effort using units of dxl is (motor stalling): %f' %(mm.get_effort_limit(motor_id, stall=True, dxl=True))[0])
    print('The max effort using units of N*m is (motor stalling): %f' %(mm.get_effort_limit(motor_id, stall=True, dxl=False))[0])
    print('The max effort using units of dxl is (motor is moving): %f' %(mm.get_effort_limit(motor_id, stall=False, dxl=True))[0])
    print('The max effort using units of N*m is (motor is moving): %f' %(mm.get_effort_limit(motor_id, stall=False, dxl=False))[0])
    max_effort = (mm.get_effort_limit(motor_id, stall=True, dxl=False))[0]

    #Test torque control function
    #Low torque test, move counterclockwise, then clockwise to the original position
    #Reset to original position (0) first
    mm.torque_control(motor_id,[0], motor_id, [2], stall=True, dxl=False)
    time.sleep(6)
    mm.torque_control(motor_id, [3.14], motor_id, [2], stall=True, dxl=False)
    time.sleep(6)
    mm.torque_control(motor_id, [0], motor_id, [2], stall=True, dxl=False)
    time.sleep(6)

    #High torque test, move counterclockwise, then clockwise to the original position
    mm.torque_control(motor_id, [0], motor_id, [2], stall=True, dxl=False)
    time.sleep(6)
    mm.torque_control(motor_id, [3.14], motor_id, [max_effort], stall=True, dxl=False)
    time.sleep(3)
    mm.torque_control(motor_id, [0], motor_id, [max_effort], stall=True, dxl=False)
    time.sleep(3)

    #Test above is repeated, but moving in the negative position

    # Low torque test, move clockwise, then counterclockwise to the original position
    # Reset to original position (0) first
    mm.torque_control(motor_id, [0], motor_id, [2], stall=True, dxl=False)
    time.sleep(6)
    mm.torque_control(motor_id, [-3.14], motor_id, [2], stall=True, dxl=False)
    time.sleep(6)
    mm.torque_control(motor_id, [0], motor_id, [2], stall=True, dxl=False)
    time.sleep(6)

    # High torque test, move clockwise, then counterclockwise to the original position
    mm.torque_control(motor_id, [0], motor_id, [2], stall=True, dxl=False)
    time.sleep(6)
    mm.torque_control(motor_id, [-3.14], motor_id, [max_effort], stall=True, dxl=False)
    time.sleep(3)
    mm.torque_control(motor_id, [0], motor_id, [max_effort], stall=True, dxl=False)
    time.sleep(3)
