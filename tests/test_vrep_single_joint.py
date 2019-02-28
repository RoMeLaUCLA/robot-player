from robot_player import MotionManager, VrepOptions

motor_id = [1]
dt = .01

with MotionManager(motor_id, dt, options=VrepOptions()) as mm:
    mm.initialize()

    for i in range(100):
        mm.set_all_goal_effort([-100], send=True)
        print(mm.get_all_present_effort())

    for i in range(100):
        mm.set_goal_effort([1], [70], send=True)
        print(mm.get_all_present_effort())