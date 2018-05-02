from __future__ import division

import argparse
import time

from numpy import matlib as np
from .vrep_interface import VrepInterface, VrepOptions
from .dxl_interface import DxlInterface, DxlOptions

def wrap_between_pi_and_neg_pi(angle):
    return (angle + np.pi) % (2*np.pi) - np.pi

class MotionManager(object):
    """
    Class to abstract commands made to a robot in simulation or in real life.
    The Motion Manager only uses one player at a time.

    This class wraps Interface classes such as the DxlInterface or VrepInterface
    in a consistent calling style so that converting between code for
    simulation and hardware experiments is easy.

    The MotionManager is initialized with an options class (for example
    vrep_options or dxl_options) Initializing the options class will help
    you set the right options for the motion manager.

    After that, passing the options class to the motion manager initializes
    it and the motion manager will take care of the rest.
    """
    def __init__(self, motor_ids, dt, options):
        """

        :param motor_ids: ids of motors
        :param dt: timestep for motors or simulator to use
        :param options: an options class: either DxlOptions or VrepOptions that lets you specify various keyword parameters.
        """

        self.sim = False
        self.dxl = False
        if isinstance(options, DxlOptions):
            self.player = 'dxl'
        elif isinstance(options, VrepOptions):
            self.player = 'vrep'
        else:
            raise ValueError("Options class invalid or missing. Specify either VrepOptions or DxlOptions to use as input to options parameter when instantiating MotionManager.")
        self.motor_id = motor_ids
        self.dt = dt
        if self.player == "vrep":
            self.sim = True
            self.device = self.vrep_init(options)

        elif self.player == 'dxl':
            self.dxl = True
            self.device = self.dxl_init(options)

    def __enter__(self):
        self.initialize()
        return self

    def __exit__(self, exception_type, exception_value, traceback):
        if isinstance(self.device, VrepInterface):
            print("Stopping VREP simulation")
            self.device.stop()
        elif isinstance(self.device, DxlInterface):
            print("Closing DXL ports")
            self.device.close()

    def initialize(self):
        # stuff to do before starting each motion set
        if isinstance(self.device, VrepInterface):
            self.device.start()
        if isinstance(self.device, DxlInterface):
            self.device.initialize()

    def dxl_init(self, options):

        # initialize dxl interface
        di = DxlInterface(options.baudrate, options.dxl_ports)
        return di

    def vrep_init(self, options):

        # initialize vrep interface
        vi = VrepInterface(self.motor_id,
                           self.dt,
                           joint_prefix=options.joint_prefix,
                           gyroscope=options.gyroscope,
                           accelerometer=options.accelerometer,
                           ft_sensor_names=options.ft_sensor_names)
        return vi

    ## Position ##
    def get_current_position(self, ids):
        # gets current position of specific joint
        # ids is a list of ids
        if self.player == 'vrep':
            return self.device.get_current_position(ids)
        if self.player == 'dxl':
            return self.device.get_current_position(ids)

    def get_all_current_position(self):
        # gets current position of robot
        return self.device.get_all_current_position()

    # TODO: refactor
    def set_goal_position(self, ids, commands, send=False):
        # ids is a list of the ids that you want to command
        # commands is a list of the values that you want to send to the actuators

        try:
            assert(len(ids) == len(commands))
        except AssertionError:
            raise ValueError('ERROR: ids and commands must be same length')

        if self.player == 'vrep':
            self.device.set_goal_position(ids, commands, send)
        if self.player == 'dxl':
            self.device.set_command_position(ids, commands)

    # TODO: refactor
    def set_all_command_position(self, command, send=False):
        # set command position for using positional arguments
        # send is whether to also trigger a timestep
        if self.player == 'vrep':
            return self.device.set_all_command_position(command, send)
        if self.player == 'dxl':
            self.device.set_all_command_position(command)

    ## Velocity ##
    # TODO: refactor
    def get_joint_velocity(self, ids):
        if self.player == 'vrep':
            return self.device.get_joint_velocity(ids)
        if self.player == 'dxl':
            return self.device.get_current_velocity(ids)

    # TODO: refactor
    def get_all_joint_velocity(self):
        if self.player == 'vrep':
            return self.device.get_all_joint_velocity()
        if self.player == 'dxl':
            raise ValueError("this function hasn't been implemented for DXL yet")  # TODO: fix this

    # TODO: refactor
    def set_joint_velocity(self, ids, commands, send=True):
        # ids is a list of the ids that you want to command
        # commands is a list of the values that you want to send to the actuators

        try:
            assert(len(ids) == len(commands))
        except AssertionError:
            raise ValueError('ERROR: ids and commands must be same length')

        if self.player == 'vrep':
            self.device.set_joint_velocity(ids, commands, send)
        if self.player == 'dxl':
            self.device.set_joint_velocity(ids, commands)

    # TODO: refactor
    def set_all_joint_velocity(self, commands, send=True):
        if self.player == 'vrep':
            self.device.set_all_joint_velocity(commands, send)
        if self.player == 'dxl':
            raise ValueError("this function hasn't been implemented for DXL yet")  # TODO: fix this

    ## Effort (force/torque) ##
    # TODO: refactor
    def set_joint_effort(self, ids, commands, send=True):

        try:
            assert(len(ids) == len(commands))
        except AssertionError:
            raise ValueError('ERROR: ids and commands must be same length')
        
        if self.player == 'vrep':
            self.device.set_joint_effort(ids, commands, send)
        if self.player == 'dxl':
            self.device.set_joint_torque(ids, commands)

    # TODO: refactor
    def set_all_joint_effort(self, commands, send=True):
        if self.player == 'vrep':
            self.device.set_all_joint_effort(commands, send)
        elif self.player == 'dxl':
            raise ValueError("this function hasn't been implemented for DXL yet")  # TODO: fix this

    # TODO: refactor
    def get_joint_effort(self, ids):
        if self.player == 'vrep':
            return self.device.get_joint_effort(ids)
        elif self.player == 'dxl':
            self.device.get_current_torque(ids)

    # TODO: refactor
    def get_all_joint_effort(self):
        if self.player == 'vrep':
            return self.device.get_all_joint_effort()
        elif self.player == 'dxl':
            raise ValueError("this function hasn't been implemented for DXL yet")  # TODO: fix this

    def wait(self, time_to_wait):
        # wait for a specified duration of time, in seconds. This command is blocking.
        if isinstance(self.device, VrepInterface):
            # timesteps to wait, rounded down to the nearest integer
            self.device.wait(int(time_to_wait/self.device.dt))
        if isinstance(self.device, DxlInterface):
            time.sleep(time_to_wait)

    def torque_on(self, ids):
        # torque on all motors, dxl specific
        if isinstance(self.device, DxlInterface):
            self.device.set_torque_enable(zip(ids, [1]*len(ids)))

    def torque_off(self, ids):
        # torque off all motors, dxl specific
        if isinstance(self.device, DxlInterface):
            self.device.set_torque_enable(zip(ids, [0]*len(ids)))

    def set_joint_ctrl_loop(self, ids, commands):
        self.device.set_joint_ctrl_loop(ids, commands)

    def get_joint_ctrl_loop(self, ids):
        return self.device.get_joint_ctrl_loop(ids)

    def advance_timestep(self):
        if isinstance(self.device, VrepInterface):
            # timesteps to wait, rounded down to the nearest integer
            self.device.wait(1)
        # if isinstance(self.device, DxlInterface):
        #     # timesteps to wait, rounded down to the nearest integer
        #     self.device.wait(dt)

def to_player_angle_offset(angles, player_offset):
    # flips joint axes around to match the player's representation. Is it's own inverse and should be called to
    # rectify each of the joint axes.
    """
    :param angles: angles (radians)
    :param player_offset: a class specifying player offsets that should be customized for each robote
    :return:
    """

    trans = player_offset.angle_trans
    offsets = player_offset.angle_offset

    return [wrap_between_pi_and_neg_pi((q + q_o)*t) for q, q_o, t in zip(angles, offsets, trans)]

def from_player_angle_offset(angles, player_offset):
    # flips joint axes around to match the kinematic representation.
    # Is it's own inverse and should be called to rectify each of the joint axes.

    """
    :param angles: angles (radians)
    :param player_offset: a class specifying player offsets that should be customized for each robot
    :return:
    """

    angles = np.asarray(angles)

    trans = player_offset.angle_trans
    offsets = player_offset.angle_offset

    angles = np.asarray([q*t - q_o for q, q_o, t in zip(angles, offsets, trans)])
    return wrap_between_pi_and_neg_pi(angles)

def to_and_from_player_effort(effort, player_offset):
    """
    Convert efforts between player and calculation
    :param effort: torque/force values
    :param player_offset: 1 or -1 for each axis
    :return:
    """
    trans = player_offset.angle_trans

    return [e*tr for e, tr in zip(effort, trans)]

def player_arg_parser(filename):
    """
    Parses command-line arguments to python interpreter.
    This allows you to easily select between using dynamixels or simulation

    :param filename:
    :return: args, which can be checked using args.dynamixel or args.simulation
    """
    parser = argparse.ArgumentParser(description=filename)
    parser.add_argument('-s', '--simulation',
                        help='run a VREP simulation',
                        action='store_true')
    parser.add_argument('-d', '--dynamixel',
                        help='use Dynamixels',
                        action='store_true')
    args = parser.parse_args()
    if not args.dynamixel and not args.simulation:
        print(
            """
            ERROR: You need to specify either a simulation or a robot to output your keyframes to."
        
            usage: python {} [-h] [-s] [-d]  
        
            optional arguments:
            -h, --help        show this help message and exit
            -s, --simulation  run a VREP simulation
            -d, --dynamixel   use Dynamixels
            """.format(filename))
        quit()

    if args.dynamixel and args.simulation:
        print("ERROR: Only select either dynamixel or simulation for now.")
        quit()

    return args

# if __name__ == "__main__":
#     # test this with the vrep file L_Arm and the right arm of the Robotis Manipulator
#     Devices = []
#     dt = .01
#     VI = VrepInterface(robot_name="", dt=dt, id_list=[9, 10, 11, 12, 13, 14, 15])
#     Devices.append()
#     Devices.append(DxlDevice([9, 10, 11, 12, 13, 14, 15]))
#
#     with MotionManager(Devices) as MM:
#         MM.initialize()
#         MM.set_all_command_position([0, 0, 0, 0, 0, 0, 0])
#         for i in xrange(100):
#             print(MM.get_all_current_position())
#             MM.wait(dt)
#
#         MM.set_all_command_position([np.pi / 10, np.pi / 10, np.pi / 10, np.pi / 10, np.pi / 10, np.pi / 10, np.pi / 10, ])
#         for i in range(100):
#             print(MM.get_all_current_position())
#             MM.wait(dt)
#
#         MM.set_all_command_position([0, 0, 0, 0, 0, 0, 0])
#         for i in range(100):
#             print(MM.get_all_current_position())
#             MM.wait(dt)
