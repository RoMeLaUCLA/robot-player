#!usr/bin/env python
__author__ = "Daniel Sun"
__email__ = "danielsun@ucla.edu"
__copyright__ = "Copyright 2017 RoMeLa"
__date__ = "Setember 5, 2017"

__version__ = "0.0.1"
__status__ = "Prototype"


import ctypes
import dynamixel_sdk as dynamixel
from dynamixel_sdk import DXL_LOBYTE, DXL_HIBYTE, DXL_LOWORD, DXL_HIWORD, DXL_MAKEDWORD, DXL_MAKEWORD
from robot_player.dxl.dxl_control_table import DXLPRO, MX106, MX106_P1, MX28, MX28_P1, XSERIES
from collections import OrderedDict
from math import pi
import subprocess
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
             MX28_P1.MX_28_P1: MX28_P1,
             XSERIES.XH430_W350: XSERIES,
             XSERIES.XH430_V350: XSERIES,
             XSERIES.XM540_W270: XSERIES}

# DEVICENAME = "/dev/ttyUSB0".encode('utf-8')  # Check which port is being used on your controller
# ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

COMM_SUCCESS = 0  # Communication Success result value
COMM_TX_FAIL = -1001  # Communication Tx Failed

def set_serial_port_low_latency(port_name):
    # sets serial port to be low latency
    subprocess.call(['setserial', port_name, 'low_latency'])

def twos_comp(val, bits):
    """compute the 2's complement of int value val"""
    if (val & (1 << (bits - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
        val = val - (1 << bits)        # compute negative value
    return val

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
            try:
                set_serial_port_low_latency(port)
            except OSError:
                print("Failed to set port {} to low latency.".format(port))


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
        elif self.motor_type == 'XSERIES':
            self.ctrl_table = XSERIES
        else:
            raise ValueError("Control table {} did not match one of the supported types: MX28, MX106, MX106_P1, DXLPRO".format(self.motor_type))


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
            d.port_handler = dynamixel.PortHandler(d.device_name)
            # initialize packet handler
            d.packet_handler = dynamixel.PacketHandler(d.protocol_version)
            self.device.append(d)
            set_serial_port_low_latency(d.device_name)
        print(self.device)



        for d in self.device:
            # Open port
            if d.port_handler.openPort() and d.port_handler.setBaudRate(self.baudrate):
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

        # print(self.id_to_port)
        print("=== Dxl_Interface initialized. ===")

    def initialize(self):
        # get ready for motion
        for d in self.device:
            self.set_torque_enable(d.motor_id, [1]*len(d.motor_id))

    def close(self):
        for d in self.device:
            d.port_handler.closePort()

    def setup_control_table(self):
        """
        setup control table and resolution of the dynamixels

        LIMITATION: all of the motors in a chain should be of the same motor type, ie. MX28, MX106, DXLPRO

        :return:
        """
        num_motors = 0
        motors_not_found = 0

        for d in self.device:
            for m_id in d.motor_id:
                num_motors += 1
                motor_model_no, dxl_comm_result, dxl_comm_error  = d.packet_handler.ping(d.port_handler, m_id)
                # dxl_comm_result = d.packet_handler.getLastTxRxResult( d.protocol_version)
                # dxl_error = d.packet_handler.getLastRxPacketError( d.protocol_version)

                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % d.packet_handler.getTxRxResult(dxl_comm_result))
                    print("Motor id " + str(m_id) + " was not found!")
                    motors_not_found += 1
                    continue
                elif dxl_comm_error != 0:
                    print("%s" % d.packet_handler.getRxPacketError(dxl_comm_error))
                else:
                    print("[ID:%03d] ping Succeeded. Dynamixel model number : %d" % (m_id, motor_model_no))


                # if dxl_comm_result != COMM_SUCCESS:
                #     print(dynamixel.getTxRxResult(d.protocol_version, dxl_comm_result))
                #     print("Motor id " + str(m_id) + " was not found!")
                #     motors_not_found += 1
                #     continue
                # elif dxl_error != 0:
                #     print(dynamixel.getRxPacketError(d.protocol_version, dxl_error))
                #
                # print("[ID:%03d] ping Succeeded. Dynamixel model number : %d" % (m_id, motor_model_no))

                # use model number to get motor object
                try:
                    ctrl_table = NUM2MODEL[motor_model_no]
                except KeyError:
                    raise TypeError('Unexpected motor type. Dynamixel model number: {}'.format(motor_model_no))
                # get attributes by assuming dictionaries (DXLPRO case) and failing down to class attribute
                try:
                    resolution = ctrl_table.resolution[motor_model_no]
                    vel_unit = ctrl_table.VEL_UNIT[motor_model_no]
                    torque_conversion = ctrl_table.TORQUE_CONVERSION[motor_model_no]
                except TypeError:
                    resolution = ctrl_table.resolution
                    vel_unit = ctrl_table.VEL_UNIT
                    torque_conversion = ctrl_table.TORQUE_CONVERSION

                d.motor[m_id] = {"model_no": motor_model_no, "ctrl_table": ctrl_table, "resolution": resolution, "vel_unit": vel_unit, 'torque_conversion': torque_conversion}
                print("motor_model_no:" + str(motor_model_no))

        # check if all motors are not found:
        if num_motors == motors_not_found:
            raise Exception("Didn't find any dynamixel motors. Verify that all motors are plugged in and that you are using the correct port.")
        # check if some motors are not found:
        if motors_not_found > 0:
            raise Exception("Not all dynamixel motors were found. Check that they are all connected and that there are no cable/wiring issues.")


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

        try:
            self.setup_group_sync_read('EFFORT_LIMIT')
            print("setup success: EFFORT_LIMIT")
        except AttributeError:
            print("setup failed: EFFORT_LIMIT")

        try:
            self.setup_group_sync_write('EFFORT_LIMIT')
            print("setup success: EFFORT_LIMIT")
        except AttributeError:
            print("setup failed: EFFORT_LIMIT")

        try:
            self.setup_group_sync_read('PRESENT_INPUT_VOLTAGE')
            print("setup success: PRESENT_INPUT_VOLTAGE")
        except AttributeError:
            print("setup failed: PRESENT_INPUT_VOLTAGE")

        try:
            self.setup_group_sync_write('PRESENT_INPUT_VOLTAGE')
            print("setup success: PRESENT_INPUT_VOLTAGE")
        except AttributeError:
            print("setup failed: PRESENT_INPUT_VOLTAGE")

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
            parameter_data_len = get_parameter_data_len(d, parameter)
            print("Configuring {} group sync write for: {}".format(parameter, d.device_name))

            # new python API way
            gsw = dynamixel.GroupSyncWrite(d.port_handler, d.packet_handler, getattr(d.ctrl_table, parameter), parameter_data_len)

            # old way
            # gw_id = d.packet_handler.groupSyncWrite(
            #                                  d.protocol_version,
            #                                  getattr(d.ctrl_table, parameter),
            #                                  parameter_data_len
            #                                  )

            # set device to have .gw_<name of parameter> attached to it for further reference
            setattr(d, "gw_" + parameter, gsw)

            for m_id in d.motor_id:
                d.motor[m_id]['LEN_{}'.format(parameter)] = parameter_data_len  # TODO needs testing

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
                # new python API
                gsr = dynamixel.GroupSyncRead(d.port_handler, d.packet_handler, getattr(d.ctrl_table, parameter), parameter_data_len)

                # old way
                # gr_id = d.packet_handler.groupSyncRead(
                #                                 d.protocol_version,
                #                                 getattr(d.ctrl_table, parameter),
                #                                 parameter_data_len)
                for m_id in d.motor_id:
                    # Add parameter storage for each Dynamixel's present position value to the Syncread storage

                    # new Python API:
                    dxl_addparam_result = gsr.addParam(m_id)

                    # old way
                    # dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncReadAddParam(gr_id, m_id)).value
                    if dxl_addparam_result != 1:
                        print("[ID:%03d] groupSyncRead addparam failed" % m_id)

                    d.motor[m_id]['LEN_{}'.format(parameter)] = parameter_data_len  # TODO needs testing

            # Protocol 1.0 doesn't have sync read, so use bulk read instead
            else:
                gr_id = d.packet_handler.groupBulkRead(
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

                    d.motor[m_id]['LEN_{}'.format(parameter)] = parameter_data_len  # TODO needs testing

            # set device to have .gw_<name of parameter> attached to it for further reference
            setattr(d, "gr_" + parameter, gsr)

    def set_torque_enable(self, ids, commands):
        # for each device, if an id in the table matches, set torque
        for d in self.device:
            ctrl_table_value = getattr(d.ctrl_table, "TORQUE_ENABLE")
            for m_id, val in zip(ids, commands):
                if m_id in d.motor_id:
                    dxl_comm_result, dxl_error = d.packet_handler.write1ByteTxRx(d.port_handler, m_id, ctrl_table_value, val)
                    # dxl_comm_result = d.packet_handler.getLastTxRxResult( d.protocol_version)
                    # dxl_error = d.packet_handler.getLastRxPacketError( d.protocol_version)
                    if dxl_comm_result != COMM_SUCCESS:
                        print(d.packet_handler.getTxRxResult(dxl_comm_result))
                    elif dxl_error != 0:
                        print(d.packet_handler.getRxPacketError(dxl_error))

    def _read_data(self, ids, address, data_length):
        """
        Read control table data from a list of ids
        :param ids:
        :param address:
        :param data_length: 1, 2 or 4 bytes. Can't do more than that.
        :return: list of the data in the same order as the ids
        """

        # choose protocol version
        result = {}
        for d in self.device:
            # choose a number of bytes to read based on data length

            device_ids = self.filter_ids(ids, d)
            for m_id in device_ids:
                if data_length == 1:
                    data, _, _ = d.packet_handler.read1ByteTxRx(d.port_handler, m_id, address)
                elif data_length == 2:
                    data, _, _ = d.packet_handler.read2ByteTxRx(d.port_handler, m_id, address)
                elif data_length == 4:
                    data, _, _ = d.packet_handler.read4ByteTxRx(d.port_handler, m_id, address)
                else:
                    print("Invalid data length: 1,2,4 bytes only")
                result[m_id] = data

        return [result[m_id] for m_id in ids] # reorder data to match original id order


    def _write_data(self, ids, address, data, data_length):
        """
        Write data to list of motor ids

        :param ids:
        :param address:
        :param data_length:
        :return:
        """

        for d in self.device:
            device_ids, commands = self.filter_ids_and_commands(ids, data, d)
            for m_id, comm in zip(device_ids, commands):
                if data_length == 1:
                    d.packet_handler.write1ByteTxRx(d.port_handler, m_id, address,  comm)
                elif data_length == 2:
                    d.packet_handler.write2ByteTxRx(d.port_handler, m_id, address,  comm)
                elif data_length == 4:
                    d.packet_handler.write4ByteTxRx(d.port_handler, m_id, address,  comm)
                else:
                    print("Invalid data length: 1,2,4 bytes only")


    def _sync_write(self, device, parameter, parameter_data_length, ids, commands, twos_complement=False):
        """
        Uses sync write to write a string of data to a single port. Use this on every port.

        :param device: one of the devices (DxlPort)
        :param parameter: string, name of ctrl table parameter
        :param parameter_data_length: length of parameter data (bytes)
        :param ids: which ids to do this for. Does no error checking to ensure ids are on the device.
        :param commands: commands to send to device. Does no type conversion beyond Python -> C
        :return:
        """
        gsw = getattr(device, "gw_" + parameter)
        for m_id, command in zip(ids, commands):
            # new Python API:
            # # TODO: DO WE NEED THIS
            # if twos_complement:
            #     parameter = twos_comp(parameter, 8*parameter_data_length)
            param_byte_list = create_byte_list(command, parameter_data_length)
            dxl_addparam_result = gsw.addParam(m_id, param_byte_list)


            # old API:
            # dxl_addparam_result = ctypes.c_ubyte(
            #     dynamixel.groupSyncWriteAddParam(getattr(device, "gw_" + parameter),
            #                                      m_id,
            #                                      command,
            #                                      parameter_data_length)).value
            if dxl_addparam_result != 1:
                print("[ID:%03d] groupSyncWrite addparam failed" % m_id)
                quit()
        # Syncwrite command
        dxl_comm_result = gsw.txPacket()

        # old way
        # dxl_comm_result = dynamixel.getLastTxRxResult(device.port_num, device.protocol_version)
        if dxl_comm_result != COMM_SUCCESS:
            print(device.packet_handler.getTxRxResult(dxl_comm_result))

        # Clear syncwrite parameter storage
        gsw.clearParam()

    def _sync_read(self, device, parameter, parameter_data_length, ids, twos_complement=False):
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
            gsr = getattr(device, "gr_" + parameter)
            dxl_comm_result = gsr.txRxPacket()
            # dxl_comm_result = dynamixel.getLastTxRxResult(device.port_num, device.protocol_version)
            if dxl_comm_result != COMM_SUCCESS:
                print(device.packet_handler.getTxRxResult(dxl_comm_result))
                return None

            # Check if groupsyncread data of all dynamixels are available:
            for m_id in ids:
                # new Python API:
                dxl_getdata_result = gsr.isAvailable(m_id, getattr(device.ctrl_table, parameter), parameter_data_length)

                # old way
                # dxl_getdata_result = ctypes.c_ubyte(
                #     gsr.groupSyncReadIsAvailable(getattr(device, "gr_" + parameter),
                #                                        m_id,
                #                                        getattr(device.ctrl_table, parameter),
                #                                        parameter_data_length)).value
                if dxl_getdata_result != 1:
                    print("[ID:%03d] groupSyncRead getdata failed" % m_id)
                    quit()

                    # Get present position value for (m_id)
                data = gsr.getData(m_id, getattr(device.ctrl_table, parameter), parameter_data_length)

                # NEEDED TO ADD THIS HERE FOR NEW PYTHON API
                if twos_complement:
                    data = twos_comp(data, parameter_data_length*8)

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

            data_list = self._sync_read(d, 'PRESENT_POSITION', d.ctrl_table.LEN_PRESENT_POSITION, id_list, twos_complement=True)
            for m_id, data in zip(d.motor_id, data_list):
                res = d.motor[m_id]["resolution"]
                pos_data.append(pos2rad(data, res))

        return pos_data

    def get_all_present_position(self):
        pos_data = []
        for d in self.device:
            data_list = self._sync_read(d, 'PRESENT_POSITION', d.ctrl_table.LEN_PRESENT_POSITION, d.motor_id, twos_complement=True)
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
            self._sync_write(d, 'GOAL_POSITION', d.ctrl_table.LEN_GOAL_POSITION, id_list, commands)

    def set_all_goal_position(self, angles):
        self.set_goal_position(self.motor_id, angles)

    def get_present_velocity(self, ids):
        vel_data = []
        for d in self.device:
            id_list = self.filter_ids(ids, d)
            data_list = self._sync_read(d, 'PRESENT_VELOCITY', d.ctrl_table.LEN_PRESENT_VELOCITY, id_list, twos_complement=True)
            for m_id, data in zip(d.motor_id, data_list):
                unit = d.motor[m_id]['vel_unit']
                vel_data.append(vel2radps(data, unit))

        return vel_data

    def set_goal_velocity(self, ids, velocities):
        for d in self.device:
            id_list, velocity_list = self.filter_ids_and_commands(ids, velocities, d)
            unit_list = [d.motor[m_id]['vel_unit'] for m_id in id_list]
            # convert radians per sec to appropriate velocity unit
            commands = [radps2vel(v, unit) for v, unit in zip(velocity_list, unit_list)]
            self._sync_write(d, 'GOAL_VELOCITY', d.ctrl_table.LEN_GOAL_VELOCITY, id_list, commands)


    def set_goal_acceleration(self, ids, accelerations):
        for d in self.device:
            id_list, acceleration_list = self.filter_ids_and_commands(ids, accelerations, d)
            #unit_list = [d.motor[m_id]['vel_unit'] for m_id in id_list]
            # convert radians per sec to appropriate velocity unit
            #commands = [radps2vel(v, unit) for v, unit in zip(velocity_list, unit_list)]
            self._sync_write(d, 'GOAL_ACCELERATION', d.ctrl_table.LEN_GOAL_ACCELERATION, id_list, acceleration_list)


    def get_present_effort(self, ids, stall, dxl):
        effort_data = []
        for d in self.device:
            id_list = self.filter_ids(ids, d)
            data_list = self._sync_read(d, 'PRESENT_EFFORT', d.ctrl_table.LEN_PRESENT_EFFORT, id_list, twos_complement=True)
            for m_id, data in zip(d.motor_id, data_list):
                ctrl_table = d.motor[m_id]['ctrl_table']
                unit = torque_conversion_equation(data, d.motor[m_id]['model_no'], True, stall, dxl, ctrl_table)
                effort_data.append(unit)

        return effort_data

    def get_present_voltage(self, ids):
        for d in self.device:
            id_list = self.filter_ids(ids, d)
            data_list = self._sync_read(d, 'PRESENT_INPUT_VOLTAGE', d.ctrl_table.LEN_PRESENT_INPUT_VOLTAGE, id_list, twos_complement=True)

        return data_list

    def get_effort_limit(self, ids, stall, dxl):
        effort_limit_data = []
        for d in self.device:
            id_list = self.filter_ids(ids, d)
            data_list = self._sync_read(d, 'EFFORT_LIMIT', d.ctrl_table.LEN_EFFORT_LIMIT, id_list)

            for m_id, data in zip(d.motor_id, data_list):
                ctrl_table = d.motor[m_id]['ctrl_table']
                unit = torque_conversion_equation(data, d.motor[m_id]['model_no'], True, stall, dxl, ctrl_table)
                effort_limit_data.append(unit)

        return effort_limit_data

    def set_goal_effort(self, ids, efforts, stall, dxl):
        for d in self.device:
            id_list, effort_list = self.filter_ids_and_commands(ids, efforts, d)
            commands = [torque_conversion_equation(data, d.motor[m_id]['model_no'], False, stall, dxl, d.motor[m_id]['ctrl_table']) for m_id, data in zip(id_list, effort_list)]
            self._sync_write(d, 'GOAL_EFFORT', d.ctrl_table.LEN_GOAL_EFFORT, id_list, commands)

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
    return int(round(rad * resolution / (2 * pi)))

def pos2rad(pos, resolution):
    return pos * (2 * pi) / resolution

def radps2vel(radps, conversion):
    # 30/pi rpm per radps, then divide by conversion to get the unit
    return int(round(radps * (30 / pi) / conversion))

def vel2radps(vel, conversion):
    # multiply by conversion to get rpm, then pi/30 radps per rpm
    return vel * conversion / (30 / pi)

def torque_conversion_equation(value, model_number, dxl_to_nm, stall, dxl, control_table):

    """
    :param value: input value from present or goal effort
    :param model_number: model number based on robotis website
    :param dxl_to_nm: True if converting from dxl units to Nm, False, if converting from Nm to dxl units
    :param stall: True if the motor is currently in stalling state, False, if the motor is currently in dynamic state
    :param dxl: True if only the direct dxl values from the motor are desired (no conversions are made)
    :param control_table: Uses the dictionary variable that contains the linear regression torque equations
    :return:

    """

    try:
        torque_equation_val = control_table.TORQUE_EQUATION_DICT[model_number]
    except:
        pass

    if dxl:
        return value

    elif dxl_to_nm and stall:
        current = value * torque_equation_val[4]
        return torque_equation_val[0] * current - torque_equation_val[1]

    elif not dxl_to_nm and stall:
        current = (value + torque_equation_val[1]) / torque_equation_val[0]
        return current * torque_equation_val[5]

    elif dxl_to_nm and not stall:
        current = value * torque_equation_val[4]
        return torque_equation_val[2] * current - torque_equation_val[3]

    elif not dxl_to_nm and not stall:
        current = (value + torque_equation_val[3]) / torque_equation_val[2]
        return current * torque_equation_val[5]

    else:
        raise Exception("Model number has not yet been verified for its torque/current conversion experimentally. Only dxl values may be received or returned.")

def get_parameter_data_len(d, parameter):
    """
    checks the model of a passed device and fetches the data length of a passed parameter. assumes that all devices on
    the same port are the same model.

    :param d: device to check for
    :param parameter: the parameter whose length is checked
    :return: command length of the parameter
    """
    # enforce assumption that all motors on one port are the same model
    motor_model_no, dxl_comm_result, dxl_comm_error = d.packet_handler.ping( d.port_handler, d.motor_id[0])

    try:  # to get motor model
        motor = NUM2MODEL[motor_model_no]
    except KeyError:
        raise TypeError('Unexpected motor type. Dynamixel model number: {}'.format(motor_model_no))

    try:  # to get command length
        parameter_data_len = getattr(motor, 'LEN_{}'.format(parameter))
    except AttributeError:
        raise AttributeError('Error: motor_model {} does not have parameter {}'.format(motor_model_no, parameter))

    return parameter_data_len


def create_byte_list(param, byte_len):
    """
    Break down a parameter into a list of either 1, 2 or 4 bytes
    :param param: the data to be broken down
    :param byte_len: number of bytes
    :return: a list containing the broken down bytes.
    """

    if byte_len == 4:
        param_list = [DXL_LOBYTE(DXL_LOWORD(param)),
                      DXL_HIBYTE(DXL_LOWORD(param)),
                      DXL_LOBYTE(DXL_HIWORD(param)),
                      DXL_HIBYTE(DXL_HIWORD(param))]
    elif byte_len == 2:
        param_list = [DXL_LOBYTE(param), DXL_HIBYTE(param)]

    elif byte_len == 1:
        param_list = [param]

    else:
        raise ValueError("byte list must be 1, 2 or 4 bytes!")

    return param_list