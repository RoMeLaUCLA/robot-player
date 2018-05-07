#!usr/bin/env python
__author__ = "Daniel Sun"
__email__ = "danielsun@ucla.edu"
__copyright__ = "Copyright 2017 RoMeLa"
__date__ = "Setember 5, 2017"

__version__ = "0.0.1"
__status__ = "Prototype"


import ctypes
from .dxl import dynamixel_functions as dynamixel
from .dxl.dxl_control_table import DXLPRO, MX106, MX106_P1, MX28, MX28_P1
from collections import OrderedDict

# conversion table. pass in the motor model number, get the object
NUM2MODEL = {MX106.MX_106: MX106,
             MX106_P1.MX_106_P1: MX106_P1,
             DXLPRO.H54_200_S500_R: DXLPRO,
             DXLPRO.H54_200_B500_R: DXLPRO,
             DXLPRO.H54_100_S500_R: DXLPRO,
             DXLPRO.H42_20_S300_R: DXLPRO,
             DXLPRO.M54_60_S250_R: DXLPRO,
             DXLPRO.M54_40_S250_R: DXLPRO,
             DXLPRO.M42_10_S260_R: DXLPRO,
             DXLPRO.L54_50_S500_R: DXLPRO,
             DXLPRO.L54_30_S500_R: DXLPRO,
             DXLPRO.L54_50_S290_R: DXLPRO,
             DXLPRO.L54_30_S400_R: DXLPRO,
             MX28.MX_28: MX28,
             MX28_P1.MX_28_P1: MX28_P1}

# DEVICENAME = "/dev/ttyUSB0".encode('utf-8')  # Check which port is being used on your controller
# ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

COMM_SUCCESS = 0  # Communication Success result value
COMM_TX_FAIL = -1001  # Communication Tx Failed

class DxlOptions(object):
    def __init__(self,
                 motor_ids,
                 motor_types,
                 ports,
                 baudrate=3000000,
                 protocol_version=2
                 ):

        """

        :param motor_ids: list of list of motor ids for each port
        :param motor_types: list of motor types for each port
        :param ports: list of strings that specify the name of the port
        :param baudrate: integer specifiying baudrate (eg. 3000000)
        :param protocol_version: integer specifying protocol to use (1 or 2)
        """
        self.dxl_ports = []
        self.baudrate = baudrate
        self.protocol_version = protocol_version

        # if motor_ids is just a single list, then put it inside another list for it
        if len(motor_ids) == 1 and not isinstance(motor_ids[0], list):
            print("motor_ids is a single list!")
            motor_ids = [motor_ids]

        for port, ids, motor_type in zip(ports, motor_ids, motor_types):
            self.dxl_ports.append(DxlPort(ids, motor_type, port, baudrate, protocol_version))

class DxlPort(object):
    def __init__(self,
                 motor_ids,
                 motor_type,
                 device_name="/dev/ttyUSB0",
                 baudrate=3000000,
                 protocol_version=2
                 ):
        self.motor_id = motor_ids
        self.motor_type = motor_type
        self.protocol_version = protocol_version
        self.device_name = device_name
        self.baudrate = baudrate
        self.motor = OrderedDict()
        self.port_num = None
        # self.gw_goalpos = None
        # self.gw_PRESENT_POSITION = None

        if self.motor_type == 'MX28':
            self.ctrl_table = MX28
        elif self.motor_type == 'MX106':
            self.ctrl_table = MX106
        elif self.motor_type == 'MX106_P1':
            self.ctrl_table = MX106_P1
        elif self.motor_type == 'DXLPRO':
            self.ctrl_table = DXLPRO

class DxlInterface(object):
    """
    Class to talk to multiple dynamixel chains, abstracted as DxlPorts

    Specify the options for each usb port like this:
    device_opts = [{'device_name':"/dev/ttyUSB0",
                    'motor_id':[1],
                    'motor_type':'DXLPRO'},
                   {'device_name':"/dev/ttyUSB1",
                    'motor_id':[2],
                    'motor_type':'DXLPRO'} ]

    This device automatically multiplexes calls to the devices to make it seem like multiple devices
    are one contiguous device.

    when you write to the motors, they will be written all at once, using the syncwrite or syncread commands.
    The order of the motors is implicit- the order that you initialized the motors in.
    First by device, then by the order that you listed the motors.

    eg. device1 = id:[1,2,3,4,5]
        device2 = id:[6,7,8,9,10]
    DI.set_all_goal_position([x,x,x,x,x,x,x,x,x,x])
    corresponds to the order 1,2,3,4,5,6,7,8,9,10

    instead of writing new functions for each of the values, access the values using the

    write, sync_write and sync_read commands

    """
    def __init__(self, baudrate, dxl_ports):
        self.baudrate = baudrate
        self.device = []

        # allocate ports
        for d in dxl_ports:
            d.port_num = dynamixel.portHandler(d.device_name)
            self.device.append(d)
        print(self.device)

        # initialize packet handler
        dynamixel.packetHandler()

        for d in self.device:
            # Open port
            if dynamixel.openPort(d.port_num) and dynamixel.setBaudRate(d.port_num, self.baudrate):
                print("Succeeded to open the port and set baudrate to {} for {}!".format(self.baudrate, d.device_name))
            else:
                print("Failed to open the port!")
                quit()

        self.setup_control_table()
        self.setup_sync_functions()

        # match up motor ids and dxl ports
        self.id_to_port = {}
        self.motor_id = []
        for d in self.device:
            for m_id in d.motor_id:
                self.id_to_port[m_id] = d
                self.motor_id.append(m_id)

        print(self.id_to_port)

    def initialize(self):
        # get ready for motion
        for d in self.device:
            self.set_torque_enable(d.motor_id, [1]*len(d.motor_id))

    def close(self):
        for d in self.device:
            dynamixel.closePort(d.port_num)

    def setup_control_table(self):
        """
        setup control table and resolution of the dynamixels

        LIMITATION: all of the motors in a chain should be of the same motor type, ie. MX28, MX106, DXLPRO

        :return:
        """
        for d in self.device:
            for m_id in d.motor_id:
                motor_model_no = dynamixel.pingGetModelNum(d.port_num, d.protocol_version, m_id)
                dxl_comm_result = dynamixel.getLastTxRxResult(d.port_num, d.protocol_version)
                dxl_error = dynamixel.getLastRxPacketError(d.port_num, d.protocol_version)
                if dxl_comm_result != COMM_SUCCESS:
                    print(dynamixel.getTxRxResult(d.protocol_version, dxl_comm_result))
                    print("Motor id " + str(m_id) + " was not found!")
                    continue
                elif dxl_error != 0:
                    print(dynamixel.getRxPacketError(d.protocol_version, dxl_error))

                print("[ID:%03d] ping Succeeded. Dynamixel model number : %d" % (m_id, motor_model_no))

                try:
                    ctrl_table = NUM2MODEL[motor_model_no]
                    resolution = ctrl_table.resolution
                except KeyError:
                    raise TypeError('Unexpected motor type. Dynamixel model number: {}'.format(motor_model_no))

                d.motor[m_id] = {"model_no": motor_model_no, "ctrl_table": ctrl_table, "resolution": resolution}
                print("motor_model_no:" + str(motor_model_no))

    def setup_sync_functions(self):
        try:
            self.setup_group_sync_write('GOAL_POSITION')
            print("setup success: GOAL_POSITION")
        except AttributeError:
            print("setup failed: GOAL_POSITION")
        try:
            self.setup_group_sync_read('PRESENT_POSITION')
            print("setup success: PRESENT_POSITION")
        except AttributeError:
            print("setup failed: PRESENT_POSITION")
        try:
            self.setup_group_sync_write('GOAL_VELOCITY')
            print("setup success: GOAL_VELOCITY")
        except AttributeError:
            print("setup failed: GOAL_VELOCITY")
        try:
            self.setup_group_sync_read('PRESENT_VELOCITY')
            print("setup success: PRESENT_VELOCITY")
        except AttributeError:
            print("setup failed: PRESENT_VELOCITY")
        try:
            self.setup_group_sync_write('GOAL_EFFORT')
            print("setup success: GOAL_EFFORT")
        except AttributeError:
            print("setup failed: GOAL_EFFORT")
        try:
            self.setup_group_sync_read('PRESENT_EFFORT')
            print("setup success: PRESENT_EFFORT")
        except AttributeError:
            print("setup failed: PRESENT_EFFORT")
            
        for d in self.device:
            print("gw_GOAL_POSITION {}".format(d.gw_GOAL_POSITION))
            print("gr_PRESENT_POSITION {}".format(d.gr_PRESENT_POSITION))

    def setup_group_sync_write(self, parameter):
        """
        Set up a group sync write parameter for later use with each dxl port
        Assign the group write ids to a DxlPort for later reference.
        :param parameter: parameter to write to, as a string
        :return:
        """
        for d in self.device:
            parameter_data_len = get_parameter_data_len(d, parameter)  # TODO: returns 4 if parameter DNE
            print("d.port_num {}".format(d.port_num))
            gw_id = dynamixel.groupSyncWrite(d.port_num,
                                             d.protocol_version,
                                             getattr(d.ctrl_table, parameter),  # TODO: fails if parameter DNE
                                             parameter_data_len
                                             )

            # set device to have .gw_<name of parameter> attached to it for further reference
            setattr(d, "gw_" + parameter, gw_id)

    def setup_group_sync_read(self, parameter):
        """
        Set up a group sync read parameter for later use with each dxl port
        Assign the group read ids to a DxlPort data member for later reference.
        :param parameter: parameter to read from, as a string
        :return:
        """

        for d in self.device:
            parameter_data_len = get_parameter_data_len(d, parameter)

            # Protocol 2.0 has sync read
            if d.protocol_version == 2:
                gr_id = dynamixel.groupSyncRead(d.port_num,
                                                d.protocol_version,
                                                getattr(d.ctrl_table, parameter),
                                                parameter_data_len)
                for m_id in d.motor_id:
                    # Add parameter storage for each Dynamixel's present position value to the Syncread storage
                    dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncReadAddParam(gr_id, m_id)).value
                    if dxl_addparam_result != 1:
                        print("[ID:%03d] groupSyncRead addparam failed" % m_id)

            # Protocol 1.0 doesn't have sync read, so use bulk read instead
            else:
                gr_id = dynamixel.groupBulkRead(d.port_num,
                                                d.protocol_version,
                                                getattr(d.ctrl_table, parameter),
                                                parameter_data_len)
                for m_id in d.motor_id:
                    # Add parameter storage for each Dynamixel's present position value to the Bulkread storage
                    dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupBulkReadAddParam(
                                                         d.group_num,
                                                         m_id,
                                                         getattr(d.ctrl_table, parameter),
                                                         parameter_data_len)
                                                         ).value
                    if dxl_addparam_result != 1:
                        print("[ID:%03d] groupBulkRead addparam failed" % m_id)
            # set device to have .gw_<name of parameter> attached to it for further reference
            setattr(d, "gr_" + parameter, gr_id)

    def set_torque_enable(self, ids, commands):
        # for each device, if an id in the table matches, set torque
        for d in self.device:
            ctrl_table_value = getattr(d.ctrl_table, "TORQUE_ENABLE")
            for m_id, val in zip(ids, commands):
                if m_id in d.motor_id:
                    dynamixel.write1ByteTxRx(d.port_num, d.protocol_version, m_id, ctrl_table_value, val)
                    dxl_comm_result = dynamixel.getLastTxRxResult(d.port_num, d.protocol_version)
                    dxl_error = dynamixel.getLastRxPacketError(d.port_num, d.protocol_version)
                    if dxl_comm_result != COMM_SUCCESS:
                        print(dynamixel.getTxRxResult(d.protocol_version, dxl_comm_result))
                    elif dxl_error != 0:
                        print(dynamixel.getRxPacketError(d.protocol_version, dxl_error))

    def _sync_write(self, device, parameter, parameter_data_length, ids, commands):
        """
        Uses sync write to write a string of data to a single port. Use this on every port.

        :param device: one of the devices (DxlPort)
        :param parameter: string, name of ctrl table parameter
        :param parameter_data_length: length of parameter data (bytes)
        :param ids: which ids to do this for. Does no error checking to ensure ids are on the device.
        :param commands: commands to send to device. Does no type conversion beyond Python -> C
        :return:
        """
        for m_id, command in zip(ids, commands):
            dxl_addparam_result = ctypes.c_ubyte(
                dynamixel.groupSyncWriteAddParam(getattr(device, "gw_" + parameter),
                                                 m_id,
                                                 command,
                                                 parameter_data_length)).value
            if dxl_addparam_result != 1:
                print("[ID:%03d] groupSyncWrite addparam failed" % m_id)
                quit()
        # Syncwrite command
        dynamixel.groupSyncWriteTxPacket(getattr(device, "gw_" + parameter))
        dxl_comm_result = dynamixel.getLastTxRxResult(device.port_num, device.protocol_version)
        if dxl_comm_result != COMM_SUCCESS:
            print(dynamixel.getTxRxResult(device.protocol_version, dxl_comm_result))

        # Clear syncwrite parameter storage
        dynamixel.groupSyncWriteClearParam(getattr(device, "gw_" + parameter))

    def _sync_read(self, device, parameter, parameter_data_length, ids):
        """
        Uses sync read and the name of a parameter to read data from ctrl table of multiple DXLs. Use this method on
        every device.

        :param device: which device to use.
        :param parameter: string, a name of the ctrl table parameter that you are using
        :param parameter_data_length: int, data length of the table, in bytes.
        :param ids: which ids to use for this. Doesn't do error checks to make sure ids are actually on the device.
        :return: data, as a list, in order of the ids.
        """

        data_list = []
        if device.protocol_version == 2:
            # Syncread present position
            dynamixel.groupSyncReadTxRxPacket(getattr(device, "gr_" + parameter))
            dxl_comm_result = dynamixel.getLastTxRxResult(device.port_num, device.protocol_version)
            if dxl_comm_result != COMM_SUCCESS:
                print(dynamixel.getTxRxResult(device.protocol_version, dxl_comm_result))
                return None

            # Check if groupsyncread data of all dynamixels are available:
            for m_id in ids:
                dxl_getdata_result = ctypes.c_ubyte(
                    dynamixel.groupSyncReadIsAvailable(getattr(device, "gr_" + parameter),
                                                       m_id,
                                                       getattr(device.ctrl_table, parameter),
                                                       parameter_data_length)).value
                if dxl_getdata_result != 1:
                    print("[ID:%03d] groupSyncRead getdata failed" % m_id)
                    quit()

                    # Get present position value for (m_id)
                data = dynamixel.groupSyncReadGetData(getattr(device, "gr_" + parameter),
                                                      m_id,
                                                      getattr(device.ctrl_table, parameter),
                                                      parameter_data_length)
                data_list.append(data)
        else:
            # Bulkread present position and moving status
            dynamixel.groupBulkReadTxRxPacket(device.group_num)
            dxl_comm_result = dynamixel.getLastTxRxResult(device.port_num, device.protocol_version)
            if dxl_comm_result != COMM_SUCCESS:
                print(dynamixel.getTxRxResult(device.protocol_version, dxl_comm_result))

            for m_id in ids:
                # Check if groupbulkread data of Dynamixels is available
                dxl_getdata_result = ctypes.c_ubyte(
                    dynamixel.groupBulkReadIsAvailable(getattr(device, "gr_" + parameter),
                                                       m_id,
                                                       getattr(device.ctrl_table, parameter),
                                                       parameter_data_length)).value
                if dxl_getdata_result != 1:
                    print("[ID:%03d] groupBulkRead getdata failed" % m_id)
                    quit()
                    # Get Dynamixel#1 present position value
                data = dynamixel.groupBulkReadGetData(getattr(device, "gr_" + parameter),
                                                      m_id,
                                                      getattr(device.ctrl_table, parameter),
                                                      parameter_data_length)
                data_list.append(data)
        return data_list

    def get_present_position(self, ids):
        pos_data = []
        for d in self.device:
            # filter out ids that are on this device
            id_list = self.filter_ids(ids, d)

            data_list = self._sync_read(d, 'PRESENT_POSITION', 4, id_list)
            for m_id, data in zip(d.motor_id, data_list):
                res = d.motor[m_id]["resolution"]
                pos_data.append(pos2rad(data, res))

        return pos_data

    def get_all_present_position(self):
        pos_data = []
        for d in self.device:
            data_list = self._sync_read(d, 'PRESENT_POSITION', 4, d.motor_id)
            for m_id, data in zip(d.motor_id, data_list):
                res = d.motor[m_id]["resolution"]
                pos_data.append(pos2rad(data, res))

        return pos_data

    def set_goal_position(self, ids, angles):
        for d in self.device:
            id_list, angle_list = self.filter_ids_and_commands(ids, angles, d)
            # convert radians to encoder counts
            res_list = [d.motor[m_id]["resolution"] for m_id in id_list]
            commands = [rad2pos(a, res) for a, res in zip(angle_list, res_list)]
            self._sync_write(d, 'GOAL_POSITION', 4, id_list, commands)

    def set_all_goal_position(self, angles):
        self.set_goal_position(self.motor_id, angles)

    def get_present_velocity(self, ids):
        vel_data = []
        for d in self.device:
            id_list = self.filter_ids(ids, d)
            vel_data.append(self._sync_read(d, 'PRESENT_VELOCITY', 4, id_list))  # TODO: check parameter_data_length of PRESENT_VELOCITY
            # TODO: parameter may be different for different motors. only listed for MX106 and DXLPRO

    def set_goal_velocity(self, ids, commands):
        for d in self.device:
            id_list, command_list = self.filter_ids_and_commands(ids, commands, d)
            self._sync_write(d, 'GOAL_VELOCITY', 4, id_list, commands)  # TODO: check parameter_data_length of GOAL_VELOCITY
            # TODO: parameter may be different for different motors. only listed for MX106 and DXLPRO

    def get_present_effort(self, ids):
        torque_data = []
        for d in self.device:
            id_list = self.filter_ids(ids, d)
            torque_data.append(self._sync_read(d, 'PRESENT_EFFORT', 4, id_list))  # TODO: check parameter_data_length of PRESSENT_CURRENT
            # TODO: parameter may be different for different motors. only listed for MX106 and DXLPRO

    def set_goal_effort(self, ids, commands):
        for d in self.device:
            id_list, command_list = self.filter_ids_and_commands(ids, commands, d)
            self._sync_write(d, 'GOAL_EFFORT', 4, id_list, commands)  # TODO: check parameter_data_length of GOAL_TORQUE
            # TODO: parameter may be different for different motors. only listed for DXLPRO, and I think it's called GOAL_CURRENT for MX106

    def filter_ids(self, ids, device):
        # filters out ids based on which ids are on the device
        """

        :param ids: motor ids to check
        :param device: device to match
        :return: list of devices that match this device
        """
        return [m_id for m_id in ids if self.id_to_port[m_id] == device]

    def filter_ids_and_commands(self, ids, commands, device):
        """

        :param ids: motor_ids to check
        :param commands: commands to attach
        :param device: device to match
        :return: (id_list, command_list) of ids and matching commands to send
        """
        id_list, comm_list = zip(*[(m_id, comm) for m_id, comm in zip(ids, commands) if self.id_to_port[m_id] == device])
        return id_list, comm_list

def rad2pos(rad, resolution):
    return int(round(rad * resolution / (2 * 3.1415926)))

def pos2rad(pos, resolution):
    return pos * (2 * 3.1415926) / resolution

def get_parameter_data_len(d, parameter):
    """
    checks the model of a passed device and fetches the data length of a passed parameter. assumes that all devices on
    the same port are the same model.

    :param d: device to check for
    :param parameter: the parameter whose length is checked
    :return: command length of the parameter
    """
    # enforce assumption that all motors on one port are the same model
    motor_model_no = dynamixel.pingGetModelNum(d.port_num, d.protocol_version, d.motor_id[0])

    try:  # to get motor model
        motor = NUM2MODEL[motor_model_no]
    except KeyError:
        raise TypeError('Unexpected motor type. Dynamixel model number: {}'.format(motor_model_no))

    try:  # to get command length
        parameter_data_len = getattr(motor, 'LEN_{}'.format(parameter))
    except AttributeError:
        raise AttributeError('Error: motor_model {} does not have parameter {}'.format(motor_model_no, parameter))

    return parameter_data_len
