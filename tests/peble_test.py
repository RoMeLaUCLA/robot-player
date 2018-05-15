"""
Example of using streaming mode for VREP getters
Test with test_redundant_chain.ttt

"""


from robot_player import MotionManager, VrepOptions
from math import sin, cos
import matplotlib.pyplot as plt
import numpy as np


ids = [1,2,3,4,5,6,7,8,9,10,11,12]
dt = .01
numtimesteps = 25

with MotionManager(ids, dt=dt, options=VrepOptions(gyroscope=True,
                                                   accelerometer=True)) as mm:
    mm.initialize()
    mm.get_present_position(ids, streaming=True)
    mm.get_present_velocity(ids, streaming=True)
    mm.read_accelerometer(streaming=True)
    mm.read_gyro(streaming=True)
    vi = mm.device

    # testing vrep generics
    vi.vrep_func_w_op('simxAddStatusbarMessage', 'This is the message to display', opmode="oneshot")

    mm.wait(.5)

    joint_position = np.zeros((numtimesteps,len(ids)))
    joint_velocity = np.zeros((numtimesteps, len(ids)))
    gyro_data = np.zeros((numtimesteps, 3))
    accel_data = np.zeros((numtimesteps, 3))
    for i in range(numtimesteps):

        if i == numtimesteps/2: # turn pid control loop off for velocity control
            mm.set_joint_ctrl_loop(ids, [False] * len(ids))

        if i < numtimesteps/2:
            mm.set_goal_position(ids, [sin(i*dt)**3+.05*m_id for m_id in ids], send=False)
        else:
            mm.set_goal_velocity(ids, [3*sin(i*dt)**3+.05*m_id for m_id in ids], send=False)

        joint_position[i,:] = np.array(mm.get_present_position(ids, buffer=True))
        joint_velocity[i, :] = np.array(mm.get_present_velocity(ids, buffer=True))
        gyro_data[i,:] = np.array(mm.read_gyro(buffer=True))
        accel_data[i,:] = np.array(mm.read_accelerometer(buffer=True))

        # vrep generic without opmode
        # print("simxGetLastCmdTime = {}".format(vi.vrep_func('simxGetLastCmdTime')))
        mm.advance_timestep()


plt.figure(1)
data_list = [joint_position, joint_velocity, gyro_data, accel_data]

for i, data in enumerate(data_list):
    plt.subplot(len(data_list),1,i+1)
    plt.plot(data)
plt.show()