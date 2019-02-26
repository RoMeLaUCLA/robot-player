from robot_player import MotionManager, VrepOptions
import numpy as np
import csv
from itertools import zip_longest
import time

motor_id = [1]
dt = .01
effort = []
position = []
times = []

with open('/home/alexander/Desktop/Acceleration_Calculation.csv', 'w', newline='') as A_Calculation:



    with MotionManager(motor_id, dt, VrepOptions()) as mm:

        start = time.time()



        for i in range(100):
            mm.set_all_goal_effort([-35])

            effort.append(mm.get_present_effort([1])[0])
            position.append(mm.get_all_present_position()[0])
            times.append(time.time() - start)

    parameters = [effort, position, times]
    export_data = zip_longest(*parameters, fillvalue= '')
    writer = csv.writer(A_Calculation)
    writer.writerow(("effort", "position(rad)", "time(s)"))
    writer.writerows(export_data)


A_Calculation.close()






        #row['position_rad']  = position
        #row['time_s'] = time



    #for i in range(100):
    #    mm.set_all_goal_effort([-25])
    #    print(mm.get_all_present_effort())

    #for i in range(100):
    #    mm.set_goal_effort([1], [70], send=True)
    #    print(mm.get_all_present_effort())

