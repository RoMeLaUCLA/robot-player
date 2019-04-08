from robot_player.mscl_imu_interface import imu_interface as mscl_imu
import time

with mscl_imu(sample_rate=500) as imu:

    for i in range(1000):

        imu_data =  imu.read()
        euler = imu_data.euler_angles
        ang_rate = imu_data.base_ang_vel
        lin_acc = imu_data.lin_acc
        # print euler
        # print euler, ang_rate
        print_string = ""
        for key, data in zip(['euler', 'ang_rate', 'lin_acc'], [euler, ang_rate, lin_acc]):
            print_string += key + " {: 4f} {: 4f} {: 4f} ".format(*data) + "| "
        print(print_string)
        time.sleep(.05)