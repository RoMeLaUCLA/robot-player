from robot_player import MotionManager, DxlOptions
import platform
from math import pi
from time import sleep


def allclose(l1, l2, tol=2.):
    for i, j in zip(l1, l2):
        if abs(i - j) > tol:
            raise AssertionError("elements of lists are {}, which is not within tolerance {}".format(abs(i - j), tol))
    return True


if platform.system() == 'Windows':
    ports = ['COM15', 'COM16']
else:
    ports = ['/dev/ttyUSB2', '/dev/ttyUSB0']

ids = [1, 2, 3, 4]
motor_ids = [(1, 2), (3, 4)]
dopts = DxlOptions(motor_ids,
                   motor_types=['MX106', 'MX106'],
                   ports=ports,
                   baudrate=3000000,
                   protocol_version=2)

with MotionManager(ids, dt=.005, options=dopts) as mm:
    mm.set_goal_velocity(ids, [pi / 2] * 4)  # test positive velocity
    sleep(4)
    allclose(mm.get_present_velocity(ids), [pi / 2] * 4, .1)
    mm.set_goal_velocity(ids, [0] * 4)

    mm.set_goal_velocity(ids, [-pi / 2] * 4)  # test negative velocity
    sleep(4)
    mm.set_goal_velocity(ids, [0] * 4)
    allclose(mm.get_present_velocity(ids), [-pi / 2] * 4, .1)

    mm.set_goal_velocity(ids, [pi, pi / 2, pi / 3, pi / 4])  # test different velocities
    sleep(4)
    allclose(mm.get_present_velocity(ids), [pi, pi / 2, pi / 3, pi / 4], .1)
    mm.set_goal_velocity(ids, [0] * 4)

    sleep(1)
    allclose(mm.get_present_velocity(ids), [0] * 4, .1)  # test zero velocity
