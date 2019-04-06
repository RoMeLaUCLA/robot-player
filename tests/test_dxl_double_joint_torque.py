from robot_player import MotionManager, DxlOptions
from robot_player.dxl.dxl_control_table import DXLPRO
import platform
import time

if platform.system() == 'Windows':
    ports = ['COM15', 'COM16']
else:
    ports = ['/dev/ttyUSB0']

motor_id = [1,2]
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
    max_effort = (mm.get_effort_limit(motor_id, stall=True, dxl=False))[0]

    mm.device.set_torque_enable(motor_id, [0] * len(motor_id))  # Set torque enable off
    mm.device._write_data(motor_id, DXLPRO.OPERATING_MODE, [3] * len(motor_id), 1)  # Set to position control mode
    print(mm.device._read_data(motor_id, DXLPRO.OPERATING_MODE, 1))
    mm.device.set_torque_enable(motor_id, [1] * len(motor_id))  # Set torque enable on


    #Test torque function
    #Low torque test, move counterclockwise, then clockwise to the original position
    #Reset to original position (0) first

    print(mm.get_present_position(motor_id))
    mm.set_goal_position(motor_id, [0,0])
    time.sleep(dt)
    mm.set_goal_effort(motor_id, [3, 3], stall=True, dxl=False)
    time.sleep(6)
    print(mm.get_present_position(motor_id))

    print(mm.get_present_position(motor_id))
    mm.set_goal_position(motor_id, [3,3])
    time.sleep(dt)
    mm.set_goal_effort(motor_id, [3, 3], stall=True, dxl=False)
    time.sleep(6)
    #print(mm.get_present_position(motor_id))

    #print(mm.get_present_position(motor_id))
    mm.set_goal_position(motor_id, [0,0])
    time.sleep(dt)
    mm.set_goal_effort(motor_id, [3, 3], stall=True, dxl=False)
    time.sleep(6)
    #print(mm.get_present_position(motor_id))


    # #High torque test, move counterclockwise, then clockwise to the original position
    mm.set_goal_effort(motor_id, [max_effort, max_effort], stall=True, dxl=False)
    time.sleep(dt)
    mm.set_goal_position(motor_id, [3,3])
    time.sleep(6)

    mm.set_goal_effort(motor_id, [max_effort, max_effort], stall=True, dxl=False)
    time.sleep(dt)
    mm.set_goal_position(motor_id, [0.01,0.01])
    time.sleep(6)


