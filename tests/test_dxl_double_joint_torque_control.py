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

    max_effort = (mm.get_effort_limit(motor_id, stall=True, dxl=False))[0]

    # Reset to zero position before starting torque control test
    mm.device.set_torque_enable(motor_id, [0] * len(motor_id))  # Set torque enable off
    mm.device._write_data(motor_id, DXLPRO.OPERATING_MODE, [3] * len(motor_id), 1)  # Set to position control mode
    print(mm.device._read_data(motor_id, DXLPRO.OPERATING_MODE, 1))
    mm.device.set_torque_enable(motor_id, [1] * len(motor_id))  # Set torque enable on

    mm.set_goal_effort(motor_id,[max_effort,max_effort],stall=True,dxl=False)
    time.sleep(dt)
    mm.set_goal_position(motor_id,[0.01,0.01])
    time.sleep(4)

    mm.device.set_torque_enable(motor_id, [0]*len(motor_id)) #Set torque enable off
    mm.device._write_data(motor_id, DXLPRO.OPERATING_MODE, [0] * len(motor_id), 1) #Set to torque control mode
    print(mm.device._read_data(motor_id, DXLPRO.OPERATING_MODE, 1))
    mm.device.set_torque_enable(motor_id, [1] * len(motor_id)) #Set torque enable on

    time.sleep(dt)

    #Low torque test = increase torque slightly, let the motor run until a certain position is reached, then go into
    #the opposite direction and return back to the zero position


    current_pos = mm.get_present_position(motor_id)
    mm.set_goal_effort(motor_id,[3,3],stall=True,dxl=False)

    start_time = time.time()

    while current_pos[0] < 6.28 or current_pos[1] < 6.28:
        current_pos = mm.get_present_position(motor_id)
        print(current_pos)
        time.sleep(dt)

    mm.set_goal_effort(motor_id,[0,0],stall=True,dxl=False)

    time.sleep(6)
    mm.set_goal_effort(motor_id,[-3,-3],stall=True,dxl=False)

    while current_pos[0] > 0.01 or current_pos[1] > 0.01:
        current_pos = mm.get_present_position(motor_id)
        print(current_pos)
        time.sleep(dt)

    mm.set_goal_effort(motor_id,[0,0],stall=True,dxl=False)
    time.sleep(10)

    # #Gradually increase the torque to high effort (10 N*m) then gradually decrease to zero torque

    current_effort = mm.get_present_effort(motor_id,stall=True,dxl=False)
    current_effort = [round(elem) for elem in current_effort]

    while current_effort[0] < 10 or current_effort[1] < 10:

        print(current_effort)

        if current_effort[0] < 10:
            current_effort[0] = current_effort[0] + 1

        if current_effort[1] < 10:
            current_effort[1] = current_effort[1] + 1

        mm.set_goal_effort(motor_id,current_effort,stall=True,dxl=False)
        time.sleep(1)

    while current_effort[0] > 0 or current_effort[1] > 0:

        print(current_effort)

        if current_effort[0] <= 10:
            current_effort[0] = current_effort[0] - 1

        if current_effort[1] <= 10:
            current_effort[1] = current_effort[1] - 1

        mm.set_goal_effort(motor_id,current_effort,stall=True,dxl=False)
        time.sleep(1)

    mm.set_goal_effort(motor_id,[0,0],stall=True,dxl=False)
    print('Returing to reference position')
    time.sleep(5)

    mm.device.set_torque_enable(motor_id, [0] * len(motor_id))
    mm.device._write_data(motor_id, DXLPRO.OPERATING_MODE, [3] * len(motor_id), 1)  # Set to Position control mode
    print(mm.device._read_data(motor_id, DXLPRO.OPERATING_MODE, 1))
    mm.device.set_torque_enable(motor_id, [1] * len(motor_id))
    mm.set_goal_effort(motor_id,[3,3],stall=True,dxl=False)
    time.sleep(dt)
    mm.set_goal_position(motor_id,[0,0])

