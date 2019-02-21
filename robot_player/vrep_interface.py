#!usr/bin/env python
__author__ = "Daniel Sun, Hosik Chae"
__email__ = "danielsun@ucla.edu, CKMagenta@gmail.com"
__copyright__ = "Copyright 2016 RoMeLa"

__version__ = "0.0.1"
__status__ = "Prototype"

'''
  This class is used to handle the communication between python and V-REP
'''

from . vrep import vrep
from collections import OrderedDict

class VrepOptions(object):
    # options class to help with creating via MotionManager
    def __init__(self, joint_prefix=None, gyroscope=False, accelerometer=False, ft_sensor_names=None, synchronous=True):
        self.joint_prefix = joint_prefix
        self.gyroscope = gyroscope
        self.accelerometer = accelerometer
        self.ft_sensor_names = ft_sensor_names
        self.opmode = None
        self.synchronous=synchronous

def error_handler(return_code, function_name):
    if return_code > 0:
        error_list = []
        if return_code & vrep.simx_return_novalue_flag:
            error_list.append("simx_return_novalue_flag: There is no command reply in the input buffer. This should not always be considered as an error, depending on the selected operation mode)")
        if return_code & vrep.simx_return_timeout_flag:
            error_list.append("simx_return_timeout_flag: The function timed out (probably the network is down or too "
                              "slow)")
        if return_code & vrep.simx_return_illegal_opmode_flag:
            error_list.append("simx_return_illegal_opmode_flag: The specified operation mode is not supported for the given function.")
        if return_code & vrep.simx_return_remote_error_flag:
            error_list.append("simx_return_remote_error_flag: The function caused an error on the server side (e.g. "
                              "an invalid handle was specified) )")
        if return_code & vrep.simx_return_split_progress_flag:
            error_list.append("simx_return_split_progress_flag: The communication thread is still processing previous split command of the same type")
        if return_code & vrep.simx_return_local_error_flag:
            error_list.append("simx_return_local_error_flag: The function caused an error on the client side")
        if return_code & vrep.simx_return_initialize_error_flag:
            error_list.append("simx_return_initialize_error_flag: simxStart was not yet called")

        for error in error_list:
            print ("VREP function {} returned {}".format(function_name, error))


class VrepInterface(object):
    """
    The VrepInterface is the main way of interacting with a simulation.

    It has commands to start/stop the simulation, control joints, set positions, print to the status bar, etc.
    An essential part of understanding how the simulation works is that objects in a simulation are accessed using their
    handles- a number that represents an object's unique id. A shape, a joint, the camera and the floor all have
    handles. By using the VrepInterface, you should not have to thnk too much about the handles or storing them.

    To be able to use this class, you need to have the files vrep.py, remoteApi.so and vrepConst.py in the same folder
    as this file. You can get these files from the VREP distribution that you downloaded, in
    <VREP root>/programming/remoteApiBindings/lib/lib/64Bit and <VREP root>/programming/remoteApiBindings/python/python
    """
    # TODO: make "set" commands not trigger the next simulation step automatically
    # TODO: make joint data structure a dictionary and not a list

    JOINT_TYPE_REVOLUTE = 10
    JOINT_TYPE_PRISMATIC = 11

    # On initialization pass in the timestep that both the python script and V-REP will be synced to
    def __init__(self, motor_id, dt, joint_prefix=None, gyroscope=False, accelerometer=False, ft_sensor_names=None,
                 opmode=vrep.simx_opmode_blocking, synchronous=True):
        vrep.simxFinish(-1)
        self._sim_Client_ID = vrep.simxStart('127.0.0.1', 19997, True, True, 5000, 1)
        if joint_prefix is None:
            self.joint_prefix = "joint"
        else:
            self.joint_prefix = joint_prefix
        self.motor_id = motor_id
        self.opmode = opmode
        self.revolute_joint_torque_control_max_speed = 1000000  # rad/s
        self.prismatic_joint_torque_control_max_speed = 100000  # m/s

        self.joint = self.get_joint_data_structure(self.motor_id)
        self.set_dt(dt)

        # sensor
        self.gyroscope = gyroscope
        self.accelerometer = accelerometer
        self.ft_sensors = {}


        if ft_sensor_names is not None:
            for ft in ft_sensor_names:
                ft_handle = self.get_object_handles(ft)
                self.ft_sensors[ft] = ft_handle

        if self._sim_Client_ID != -1:
            print("Connected to V-REP remote API server.")
            # Setup synchronized simulation
            vrep.simxSynchronous(self._sim_Client_ID, synchronous)

        else:
            # Failed to connect to remote API server
            raise SimulationConfigOffError("Please double check whether the simulator is running.")

    def create_dummy(self, size, color, position):
        #position is a 3 value list in the format [x,y,z]
        #colors: 4*3 bytes (0-255) for ambient_diffuse RGB, 3 reserved values (set to zero), specular RGB and emissive RGB
        errorCode, handle_dummy = vrep.simxCreateDummy(self._sim_Client_ID, size, color, vrep.simx_opmode_oneshot_wait)
        vrep.simxSetObjectPosition(self._sim_Client_ID, handle_dummy, -1, position, vrep.simx_opmode_oneshot_wait)


    def sendSignalFootstepPoints(self, position):
        #position is of form [x,y,z]
        position_string = vrep.simxPackFloats(position)
        vrep.simxWriteStringStream(self._sim_Client_ID, 'FootstepPoint', position_string, vrep.simx_opmode_streaming)

    def stopSignalFootstepPoints(self):
        vrep.simxWriteStringStream(self._sim_Client_ID, 'FootstepPoint', "end", vrep.simx_opmode_discontinue)

    def receiveGyroSignal(self, oneshot=False, firstTime=True):
        if oneshot:
            err, dat = vrep.simxGetStringSignal(self._sim_Client_ID, 'gyro_body', vrep.simx_opmode_oneshot_wait)
            if err == 0:
                vrep.simxClearStringSignal(self._sim_Client_ID, 'gyro_body', vrep.simx_opmode_oneshot_wait)
                gyro_data = vrep.simxUnpackFloats(dat)
                return gyro_data
            else:
                return "Empty"
        else:
            if firstTime == True:
                err, dat = vrep.simxGetStringSignal(self._sim_Client_ID, 'gyro_body', vrep.simx_opmode_streaming)
                if err==0:
                    vrep.simxClearStringSignal(self._sim_Client_ID, 'gyro_body', vrep.simx_opmode_streaming)
                    gyro_data = vrep.simxUnpackFloats(dat)
                    return gyro_data
                else:
                    return "Empty"
            else:
                err, dat = vrep.simxGetStringSignal(self._sim_Client_ID, 'gyro_body', vrep.simx_opmode_buffer)
                if err == 0:
                    vrep.simxClearStringSignal(self._sim_Client_ID, 'gyro_body', vrep.simx_opmode_buffer)
                    gyro_data = vrep.simxUnpackFloats(dat)
                    return gyro_data
                else:
                    return "Empty"

    def receiveToePosSignal(self):
        err, dat = vrep.simxGetStringSignal(self._sim_Client_ID, 'toe_pos', vrep.simx_opmode_oneshot_wait)
        if err == 0:
            vrep.simxClearStringSignal(self._sim_Client_ID, 'toe_pos', vrep.simx_opmode_oneshot_wait)
            toe_pos = vrep.simxUnpackFloats(dat)
            return toe_pos
        else:
            return "Empty"

    def receiveGPSSignal(self):
        err, dat = vrep.simxGetStringSignal(self._sim_Client_ID, 'GPS', vrep.simx_opmode_oneshot_wait)
        if err==0:
            vrep.simxClearStringSignal(self._sim_Client_ID, 'GPS', vrep.simx_opmode_oneshot_wait)
            GPS = vrep.simxUnpackFloats(dat)
            return GPS
        else:
            return "Empty"

    def set_dt(self, dt):
        self.dt = dt
        vrep.simxSetFloatingParameter(self._sim_Client_ID, vrep.sim_floatparam_simulation_time_step, self.dt,
                                      vrep.simx_opmode_oneshot)

    def start(self):
        # Start the simulation
        vrep.simxStartSimulation(self._sim_Client_ID, vrep.simx_opmode_oneshot)
        if self.gyroscope:
            self.read_gyro(initialize=True)  # (operationMode=vrep.simx_opmode_blocking)
        if self.accelerometer:
            self.read_accelerometer(initialize=True)  # (operationMode=vrep.simx_opmode_blocking)

    def stop(self):
        print("Ending communication with VREP...")
        self.print_to_statusbar("Ending communication with client " + str(self._sim_Client_ID))
        vrep.simxStopSimulation(self._sim_Client_ID, vrep.simx_opmode_blocking)
        vrep.simxFinish(-1)

    def print_to_statusbar(self, msg):
        vrep.simxAddStatusbarMessage(self._sim_Client_ID, msg, vrep.simx_opmode_oneshot)

    def wait(self, timesteps_to_wait):
        for i in range(timesteps_to_wait):
            self.send_command()

    def get_ping_time(self):
        _, ping_time = vrep.simxGetPingTime(self._sim_Client_ID)
        error_handler(_, self.get_ping_time.__name__)
        return ping_time

    def get_object_position(self, object_handle, base_handle=-1, **kwargs):
        if kwargs.get('streaming'):
            opmode = vrep.simx_opmode_streaming
        elif kwargs.get('buffer'):
            opmode = vrep.simx_opmode_buffer
        else:
            opmode = vrep.simx_opmode_blocking
        _, data =  vrep.simxGetObjectPosition(self._sim_Client_ID, object_handle, base_handle, opmode)

        if _ != 0:
            error_handler(_, self.get_object_orientation.__name__)
        return data

    def set_object_position(self, position, object_handle, base_handle=-1):
        return vrep.simxSetObjectPosition(
            self._sim_Client_ID, object_handle, base_handle, position, vrep.simx_opmode_oneshot)

    def get_object_orientation(self, object_handle, base_handle=-1, **kwargs):
        if kwargs.get('streaming'):
            opmode = vrep.simx_opmode_streaming
        elif kwargs.get('buffer'):
            opmode = vrep.simx_opmode_buffer
        else:
            opmode = vrep.simx_opmode_blocking
        _, data = vrep.simxGetObjectOrientation(self._sim_Client_ID, object_handle, base_handle, opmode)

        if _ != 0:
            error_handler(_, self.get_object_orientation.__name__)
        return data

    def set_object_orientation(self, orientation, object_handle, base_handle=-1):
        return vrep.simxSetObjectOrientation(
            self._sim_Client_ID, object_handle, base_handle, orientation, vrep.simx_opmode_oneshot)

    def get_object_handles(self, object_name):
        return vrep.simxGetObjectHandle(self._sim_Client_ID, object_name, vrep.simx_opmode_blocking)[1]

    """
    Joint Communication functions

    These functions use the convention that the joint parameter is a list of dictionaries.
    these commands should have identical syntax between Dynamixel and VREP.

    """

    def get_joint_handles(self):
        # TODO: not sure if this automatically comes back in the order that you want.
        returnCode, handles, intData, floatData, stringData = vrep.simxGetObjectGroupData(self._sim_Client_ID,
                                                                                          vrep.sim_object_joint_type, 0,
                                                                                          vrep.simx_opmode_blocking)
        self._sim_joint_handles = dict(zip(stringData, handles))

        return self._sim_joint_handles

    def get_joint_data_structure(self, motor_ids):
        """
        Creates a joint data structure, common thing to do before starting a simulation.
        :param motor_ids: ids to look for
        :return: dictionary that has joint data. keys are the motor_ids that match the names of joints in the
        simulation.
        """

        # get joint names and handles
        joint_names_and_handles = self.get_joint_handles()

        # get joint datas
        returnCode, handles, intData, floatData, _ = vrep.simxGetObjectGroupData(self._sim_Client_ID,
                                                                                 vrep.sim_object_joint_type, 16,
                                                                                 vrep.simx_opmode_blocking)

        # create tuples
        joint_type_list = []
        joint_mode_list = []
        joint_limit_low_list = []
        joint_range_list = []

        for h in handles:
            joint_type_list.append(intData.pop(0))
            joint_mode_list.append(intData.pop(0))
            joint_limit_low_list.append(floatData.pop(0))
            joint_range_list.append(floatData.pop(0))

        # first, organize joint_data by handle using dictionary comprehension
        joint_data_by_handle = {h: {'joint_type': jtype, 'joint_mode': mode, 'limit_low': limit_low, 'range': joint_range}
                                for h, jtype, mode, limit_low, joint_range in
                                zip(handles, joint_type_list, joint_mode_list, joint_limit_low_list, joint_range_list)}

        # reorganize by joint id
        joint_data = []
        for name, h in joint_names_and_handles.items():
            # update dictionary with additional info
            joint_data_by_handle[h].update({'sim_handle': h,
                                            'sim_name': name,
                                            'id': int(name[len(self.joint_prefix):])})
            joint_data.append(joint_data_by_handle[h])

        # filter for specified joints
        new_joint = [entry for entry in joint_data if entry['id'] in motor_ids]

        # sort joints to be in order
        new_joint.sort(key=lambda jt_dict: int(jt_dict['id']))

        # setup joint velocities
        for j in new_joint:
            if j['joint_type'] == VrepInterface.JOINT_TYPE_REVOLUTE:
                print("revolute_joint")
                j['joint_target_velocity'] = self.revolute_joint_torque_control_max_speed
            elif j['joint_type'] == VrepInterface.JOINT_TYPE_PRISMATIC:
                print("prismatic_joint")
                j['joint_target_velocity'] = self.prismatic_joint_torque_control_max_speed
            else:
                raise Exception("can't control a joint that isn't revolute or prismatic")

        # create dictionary that preserves order
        joint_dict = OrderedDict()
        for j in new_joint:
            joint_dict[j['id']] = j

        return joint_dict

    ## Position ##
    def get_present_position(self, ids, **kwargs):
        if kwargs.get('streaming'):
            opmode = vrep.simx_opmode_streaming #+ int(self.dt*1000)
        elif kwargs.get('buffer'):
            opmode = vrep.simx_opmode_buffer
        else:
            opmode = vrep.simx_opmode_blocking

        joint_angles = []
        for i in ids:
            _, data = vrep.simxGetJointPosition(self._sim_Client_ID, self.joint[i]['sim_handle'],
                                  operationMode=opmode)
            error_handler(_, __name__)
            joint_angles.append(data)

        return joint_angles

    def get_all_present_position(self, **kwargs):
        # use the built in joint data structure to read all motor positions
        # commands must be for joints ordered from least to greatest
        return self.get_present_position(self.motor_id, **kwargs)

    def set_goal_position(self, ids, commands, send=False):
        vrep.simxPauseCommunication(self._sim_Client_ID, True)
        for i, c in zip(ids, commands):
            vrep.simxSetJointTargetPosition(self._sim_Client_ID, self.joint[i]['sim_handle'], c,
                                            vrep.simx_opmode_oneshot)
        vrep.simxPauseCommunication(self._sim_Client_ID, False)
        if send:
            self.send_command()

    def set_joint_position(self, ids, commands):
        for i, c in zip(ids, commands):
            j = self.joint[i]
            vrep.simxSetJointPosition(self._sim_Client_ID, j['sim_handle'], c,
                                      vrep.simx_opmode_oneshot)

    def set_all_goal_position(self, commands, send_command=True):
        # use the built in joint data structure to write commands to all joints.
        # commands must be for joints ordered from least to greatest ie. (1,2,3 ... n)
        self.set_goal_position(self.motor_id, commands, send_command)

    ## Velocity ##
    def get_present_velocity(self, ids, **kwargs):
        if kwargs.get('streaming'):
            opmode = vrep.simx_opmode_streaming
        elif kwargs.get('buffer'):
            opmode = vrep.simx_opmode_buffer
        else:
            opmode = vrep.simx_opmode_blocking
        joint_velocity = []
        for i in ids:
            j = self.joint[i]
            _, vel = vrep.simxGetObjectFloatParameter(self._sim_Client_ID, j['sim_handle'], vrep.sim_jointfloatparam_velocity, opmode)
            if _ > 1:
                error_handler(_,self.get_present_velocity.__name__ )
                raise Exception("VREP Error {}".format(_))
            else:
                joint_velocity.append(vel)
        return joint_velocity

    def get_all_present_velocity(self, **kwargs):
        # use the built in joint data structure to read all motor velocities
        # commands must be for joints ordered from least to greatest
        return self.get_present_velocity(self.motor_id, **kwargs)

    def set_goal_velocity(self, ids, commands, send=True):
        for i, c in zip(ids, commands):
            j = self.joint[i]
            vrep.simxSetJointTargetVelocity(self._sim_Client_ID, j['sim_handle'], c, vrep.simx_opmode_oneshot)
        if send:
            self.send_command()

    def set_all_goal_velocity(self, commands, send_command=True):
        # use the built in joint data structure to write commands to all joints.
        # commands must be for joints ordered from least to greatest ie. (1,2,3 ... n)
        self.set_goal_velocity(self.joint, commands, send_command)

    ## Effort (force/torque/PWM/current) ##
    def get_present_effort(self, ids, **kwargs):
        if kwargs.get('streaming'):
            opmode = vrep.simx_opmode_streaming
        elif kwargs.get('buffer'):
            opmode = vrep.simx_opmode_buffer
        else:
            opmode = vrep.simx_opmode_blocking
        effort_list = []
        for i in ids:
            j = self.joint[i]
            _, effort = vrep.simxGetJointForce(self._sim_Client_ID, j['sim_handle'], opmode)
            if _ > 1:
                raise Exception("Return code non-zero: {}".format(_))
            effort_list.append(effort)
        return effort_list

    def get_all_present_effort(self, **kwargs):
        return self.get_present_effort(self.joint, **kwargs)

    def set_goal_effort(self, ids, commands, send=True):
        for i, c in zip(ids, commands):
            # set joint speed
            j = self.joint[i]
            joint_effort = self.get_present_effort([i])[0]  # get the value in the list

            # if either the sign of the joint effort or the direction of the command change,
            # flip the sign of the target velocity
            if sign(joint_effort)*sign(c) < 0:
                j['joint_target_velocity'] = -1 * j['joint_target_velocity']
                vrep.simxSetJointTargetVelocity(self._sim_Client_ID, j['sim_handle'], j['joint_target_velocity'], vrep.simx_opmode_blocking)
            vrep.simxSetJointForce(self._sim_Client_ID, j['sim_handle'], abs(c), vrep.simx_opmode_blocking)

        if send:
            self.send_command()

    def set_all_goal_effort(self, commands, send=True):
        # print("setting joint effort")
        self.set_goal_effort(self.motor_id, commands, send)

    ## Control Loop ##
    def get_joint_ctrl_loop(self, ids):
        enable_list = []
        for i in ids:
            j = self.joint[i]
            # convert True/False to ints
            _, val = vrep.simxGetObjectIntParameter(self._sim_Client_ID, j['sim_handle'], vrep.sim_jointintparam_ctrl_enabled, vrep.simx_opmode_blocking)
            if _ > 1:
                raise Exception("VREP Error {}".format(_))
            else:
                enable_list.append(val)

        return enable_list

    def set_joint_ctrl_loop(self, ids, commands):
        for i, c in zip(ids, commands):

            j = self.joint[i]
            # convert True/False to ints
            vrep.simxSetObjectIntParameter(self._sim_Client_ID, j['sim_handle'], vrep.sim_jointintparam_ctrl_enabled, int(c), vrep.simx_opmode_blocking)

    def send_command(self):
        vrep.simxSynchronousTrigger(self._sim_Client_ID)
        # vrep.simxGetPingTime(self._sim_Client_ID)

    """
    Sensor Communication functions

    These functions are for communicating with sensors in VREP. They utilize the custom versions of the accelerometer
    and gyro defined in the RobotData models
    """

    def read_gyro(self, **kwargs):
        if kwargs.get('streaming'):
            opmode = vrep.simx_opmode_streaming
        elif kwargs.get('buffer'):
            opmode = vrep.simx_opmode_buffer
        else:
            opmode = vrep.simx_opmode_blocking
        returnCode, _, outFloats, _, _ = vrep.simxCallScriptFunction(self._sim_Client_ID,
                                                                     scriptDescription="GyroSensor",
                                                                     options=vrep.sim_scripttype_childscript,
                                                                     functionName="getGyroData",
                                                                     inputInts=[],
                                                                     inputFloats=[],
                                                                     inputStrings="",
                                                                     inputBuffer="",
                                                                     operationMode=opmode)
        # if returnCode != vrep.simx_return_ok:
        #     raise Exception("ERROR in {}: returnCode = {}".format(__name__, returnCode))

        return outFloats

    def read_accelerometer(self, **kwargs):
        if kwargs.get('streaming'):
            opmode = vrep.simx_opmode_streaming
        elif kwargs.get('buffer'):
            opmode = vrep.simx_opmode_buffer
        else:
            opmode = vrep.simx_opmode_blocking
        returnCode, _, outFloats, _, _ = vrep.simxCallScriptFunction(self._sim_Client_ID,
                                                                     scriptDescription="Accelerometer",
                                                                     options=vrep.sim_scripttype_childscript,
                                                                     functionName="getAccelData",
                                                                     inputInts=[],
                                                                     inputFloats=[],
                                                                     inputStrings="",
                                                                     inputBuffer="",
                                                                     operationMode=opmode)
        if returnCode != vrep.simx_return_ok:
            error_handler(returnCode, self.read_accelerometer.__name__)
        return outFloats

    def read_ft_sensor(self, sensor_id, **kwargs):
        if kwargs.get('streaming'):
            opmode = vrep.simx_opmode_streaming
        elif kwargs.get('buffer'):
            opmode = vrep.simx_opmode_buffer
        else:
            opmode = vrep.simx_opmode_blocking
        returnCode, state, forceVector, torqueVector = vrep.simxReadForceSensor(self._sim_Client_ID,
                                                                                self.ft_sensors[sensor_id],
                                                                                opmode)
        if returnCode != vrep.simx_return_ok:
            error_handler(returnCode, self.read_ft_sensor.__name__)
        return forceVector, torqueVector

    """
    Generics
    
    These functions are to allow generic access to all of VREP's functions by specifying the name of the function
    """
    def vrep_func(self, fn_name, *args):
        """
        Generic VREP function access. Automatically supplies the client ID. Everything else is left to the user to
        supply the correct arguments.
        :param fn_name: string with the name of the function
        :param args: arguments for the function, in order, skipping the client ID
        :return:
        """

        func_to_call = getattr(vrep, fn_name)
        return func_to_call(self._sim_Client_ID, *args)

    def vrep_func_w_op(self, fn_name, *args, **kwargs):
        """
        Generic VREP function access. Automatically supplies the client ID and the last parameter, which is the
        operation mode. Everything else is left to the user to supply the correct arguments.
        :param fn_name: string with the name of the function
        :param args: arguments for the function, in order
        :param kwargs:
        :return:
        """

        if kwargs.get('streaming'):
            opmode = vrep.simx_opmode_streaming
        elif kwargs.get('buffer'):
            opmode = vrep.simx_opmode_buffer
        elif kwargs.get('oneshot'):
            opmode = vrep.simx_opmode_oneshot
        elif kwargs.get('blocking'):
            opmode = vrep.simx_opmode_blocking
        else:
            opmode = vrep.simx_opmode_blocking

        # append opmode to arguments
        args = list(args)
        args.append(opmode)
        func_to_call = getattr(vrep, fn_name)
        return func_to_call(self._sim_Client_ID, *args)

def sign(x):
    if x > 0:
        return 1
    elif x < 0:
        return -1
    else:
        return 0

class SimulationConfigOffError(Exception):
    """
    If there is a problem with a configuration in the simulation
    """
    pass
