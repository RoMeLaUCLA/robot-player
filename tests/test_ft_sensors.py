from robot_player import VrepOptions, MotionManager

ft_sensor_names = ['Force_sensor','Force_sensor0']
motor_ids = [1]
options = VrepOptions(ft_sensor_names=ft_sensor_names)
with MotionManager(motor_ids=motor_ids,dt=.005, options=options) as mm:
    mm.initialize()

    print mm.read_ft_sensor(ft_sensor_names[0], initialize=True)
    print mm.read_ft_sensor(ft_sensor_names[1], initialize=True)
    for i in range(1000):
        print mm.read_ft_sensor(ft_sensor_names[0], initialize=False),
        print mm.read_ft_sensor(ft_sensor_names[1], initialize=False)
        mm.advance_timestep()
