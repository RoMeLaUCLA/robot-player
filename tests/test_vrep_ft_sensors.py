"""
Test the ft sensors with VREP, as well as the gyro and the accelerometer. Need to add readouts for the F/T sensors though.
"""


from robot_player import VrepOptions, MotionManager
import matplotlib.pyplot as plt
import numpy as np

ft_sensor_names = ['Force_sensor','Force_sensor0']
motor_ids = [1]
dt = .005

options = VrepOptions(ft_sensor_names=ft_sensor_names, gyroscope=True, accelerometer=True)
with MotionManager(motor_ids=motor_ids,dt=dt, options=options) as mm:
    mm.initialize()

    print(mm.read_ft_sensor(ft_sensor_names[0], initialize=True))
    print(mm.read_ft_sensor(ft_sensor_names[1], initialize=True))
    print(mm.read_imu(initialize=True))


    rpy_list = []
    acc_list = []
    ft1_list = []
    ft2_list = []
    times = np.arange(100)*dt

    for i in times:
        ft1_list.append(mm.read_ft_sensor(ft_sensor_names[0], initialize=False))
        ft2_list.append(mm.read_ft_sensor(ft_sensor_names[1], initialize=False))
        rpy, acc = mm.read_imu(initialize=False)
        rpy_list.append(rpy)
        acc_list.append(acc)
        mm.advance_timestep()

times = times[1:]

plt.figure(1)
plt.suptitle('roll pitch yaw')
for i, series in enumerate(zip(*rpy_list[1:])):
    plt.subplot(3,1,i+1)
    plt.plot(times,series)

plt.figure(2)
plt.suptitle('acceleration x y z')
for i, series in enumerate(zip(*acc_list[1:])):
    plt.subplot(3,1,i+1)
    plt.plot(times,series)

# plt.figure(3)
# plt.suptitle('ft_sensors')
# for i, series in enumerate([ft1_list[1:], ft2_list[1:]]):
#     plt.subplot(3,1,i+1)
#     plt.plot(times,series)


# plt.plot(zip(times, rpy_list))
plt.show()