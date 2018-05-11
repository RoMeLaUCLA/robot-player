"""
Example of using streaming mode for VREP getters

"""


from robot_player import MotionManager, VrepOptions
from math import sin, cos
import matplotlib.pyplot as plt
import numpy as np

ids = [0,1,2,3,4,5,6,7]
dt = .01
numtimesteps = 500

with MotionManager(ids, dt=dt, options=VrepOptions(joint_prefix='revolute_joint',
                                                   gyroscope=True,
                                                   accelerometer=True)) as mm:
    mm.initialize()
    mm.get_present_position(ids, streaming=True)
    mm.get_present_velocity(ids, streaming=True)
    mm.read_accelerometer(streaming=True)
    mm.read_gyro(streaming=True)

    mm.wait(.5)

    joint_position = np.zeros((numtimesteps,len(ids)))
    joint_velocity = np.zeros((numtimesteps, len(ids)))
    gyro_data = np.zeros((numtimesteps, 3))
    accel_data = np.zeros((numtimesteps, 3))
    for i in range(numtimesteps):

        if i == numtimesteps/2: # turn pid control loop off for velocity control
            mm.set_joint_ctrl_loop(ids, [False] * len(ids))

        if i < numtimesteps/2:
            mm.set_goal_position(ids, [sin(i*dt)**3+.05*m_id for m_id in ids], send=True)
        else:
            mm.set_goal_velocity(ids, [3*sin(i*dt)**3+.05*m_id for m_id in ids], send=True)

        joint_position[i,:] = np.array(mm.get_present_position(ids, buffer=True))
        joint_velocity[i, :] = np.array(mm.get_present_velocity(ids, buffer=True))
        gyro_data[i,:] = np.array(mm.read_gyro(buffer=True))
        accel_data[i,:] = np.array(mm.read_accelerometer(buffer=True))


plt.figure(1)
data_list = [joint_position, joint_velocity, gyro_data, accel_data]

for i, data in enumerate(data_list):
    plt.subplot(len(data_list),1,i+1)
    plt.plot(data)
plt.show()