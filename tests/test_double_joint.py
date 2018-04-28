from robot_player import MotionManager, VrepOptions
import numpy as np
from time import sleep

motor_id = [1,2]
dt = .01

with MotionManager(motor_id, dt, VrepOptions(joint_prefix="joint")) as mm:

    for i in range(100):
        assert(mm.get_current_position([1,2]) == mm.get_all_current_position())
        # send command to just one motor
        mm.set_goal_position([1], [1])
        mm.set_goal_position([2], [1])
        mm.advance_timestep()
    # check that motors got to right place
    pos = mm.get_all_current_position()
    assert(np.allclose(pos,[1,1]))

    for i in range(100):
        mm.set_all_command_position([-1,-2])
        mm.advance_timestep()
    # check that motors got to right place
    pos = mm.get_all_current_position()
    assert(np.allclose(pos,[-1,-2]))

    # joint velocity
    print(mm.get_joint_velocity([1,2]))
    assert(mm.get_joint_velocity([1,2]) == mm.get_all_joint_velocity())

    # switch to force controlled tests
    mm.set_joint_ctrl_loop([1,2],[False, True])
    assert(mm.get_joint_ctrl_loop([1,2])== [0,1])



    mm.set_joint_ctrl_loop([1, 2], [False, False])
    print("set joint velocity")
    for i in range(100):
        mm.set_joint_velocity([1,2], [1,-1])
        mm.advance_timestep()
    print("set all joint velocity")
    for i in range(100):
        mm.set_all_joint_velocity([1,2], [2,-2])
        mm.advance_timestep()

    print("reset position")
    mm.set_joint_ctrl_loop([1, 2], [True, True])
    mm.set_all_joint_effort([100, 100], send=False)
    for i in range(1000):
        mm.set_all_command_position([0,0])
        mm.advance_timestep()

    mm.set_joint_ctrl_loop([1, 2], [False, False])
    print("test set_joint_effort")
    for i in range(100):
        # get joint effort is also run in the set_all_joint_effort part
        mm.set_all_joint_effort([.25,0])
        mm.advance_timestep()
    for i in range(100):
        mm.set_joint_effort([1,2], [0,0])
        mm.advance_timestep()




