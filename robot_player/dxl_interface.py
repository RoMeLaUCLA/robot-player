#!usr/bin/env python
__author__ = "Daniel Sun"
__email__ = "danielsun@ucla.edu"
__copyright__ = "Copyright 2017 RoMeLa"
__date__ = "Setember 5, 2017"

__version__ = "0.0.1"
__status__ = "Prototype"


import ctypes
import time

import dxl.dynamixel_functions as dynamixel
from dxl.dxl_control_table import DXLPRO, MX106, MX106_P1, MX28

# Settings TODO: move this to a configuration file
# Control table address
ADDR_PRO_TORQUE_ENABLE = DXLPRO.TORQUE_ENABLE  # Control table address is different in Dynamixel model
ADDR_PRO_GOAL_POSITION = DXLPRO.GOAL_POSITION
ADDR_PRO_PRESENT_POSITION = DXLPRO.PRESENT_POSITION

# data Byte Length
LEN_GOAL_POSITION = 4
LEN_PRESENT_POSITION = 4

# Protocol version
PROTOCOL_VERSION = 2  # See which protocol version is used in the Dynamixel

# Default settings
BAUDRATE = 3000000
# DEVICENAME = "/dev/ttyUSB0".encode('utf-8')  # Check which port is being used on your controller
# ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

COMM_SUCCESS = 0  # Communication Success result value
COMM_TX_FAIL = -1001  # Communication Tx Failed

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
        self.motor = {}
        self.port_num = None
        self.gw_goalpos = None
        self.gr_prespos = None

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
    DI.set_all_command_position([x,x,x,x,x,x,x,x,x,x])
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
        print self.device

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

    # turn on motors and set up group sync read/write

    def initialize(self):
        # get ready for motion
        for d in self.device:
            self.set_torque_enable(zip(d.motor_id, [1]*len(d.motor_id)))

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
                motor_model_no = dynamixel.pingGetModelNum(d.port_num, d.protocol_version , m_id)
                dxl_comm_result = dynamixel.getLastTxRxResult(d.port_num, d.protocol_version )
                dxl_error = dynamixel.getLastRxPacketError(d.port_num, d.protocol_version )
                if dxl_comm_result != COMM_SUCCESS:
                    print(dynamixel.getTxRxResult(d.protocol_version , dxl_comm_result))
                    print "Motor id " + str(m_id) + " was not found!"
                    continue
                elif dxl_error != 0:
                    print(dynamixel.getRxPacketError(d.protocol_version , dxl_error))

                print("[ID:%03d] ping Succeeded. Dynamixel model number : %d" % (m_id, motor_model_no))

                if motor_model_no == MX106.MX_106:
                    ctrl_table = MX106
                    resolution = MX106.resolution

                if motor_model_no == MX106_P1.MX_106_P1:
                    ctrl_table = MX106_P1
                    resolution = MX106_P1.resolution

                elif motor_model_no in [DXLPRO.H54_200_S500_R,
                                        DXLPRO.H54_200_B500_R,
                                        DXLPRO.H54_100_S500_R,
                                        DXLPRO.H42_20_S300_R,
                                        DXLPRO.M54_60_S250_R,
                                        DXLPRO.M54_40_S250_R,
                                        DXLPRO.M42_10_S260_R,
                                        DXLPRO.L54_50_S500_R,
                                        DXLPRO.L54_30_S500_R,
                                        DXLPRO.L54_50_S290_R,
                                        DXLPRO.L54_30_S400_R,
                                        ]:
                    print DXLPRO

                    ctrl_table = DXLPRO
                    resolution = DXLPRO.resolution[motor_model_no]

                elif motor_model_no == MX28.MX_28:
                    ctrl_table = MX28
                    resolution = MX28.resolution
                else:
                    print "motor_model not found!"
                    quit()
                d.motor[m_id] = {"model_no":motor_model_no, "ctrl_table":ctrl_table, "resolution":resolution}
                print "motor_model_no:" + str(motor_model_no)

    def setup_sync_functions(self):
        for d in self.device:
            print "d.port_num {}".format(d.port_num)
            d.gw_goalpos = dynamixel.groupSyncWrite(d.port_num, d.protocol_version, d.ctrl_table.GOAL_POSITION, d.ctrl_table.LEN_GOAL_POSITION)

            # Protocol 2.0 has sync read
            if d.protocol_version == 2:
                d.gr_prespos = dynamixel.groupSyncRead(d.port_num, d.protocol_version, d.ctrl_table.PRESENT_POSITION,
                                                       d.ctrl_table.LEN_PRESENT_POSITION)
                for m_id in d.motor_id:
                    # Add parameter storage for each Dynamixel's present position value to the Syncread storage
                    dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncReadAddParam(d.gr_prespos, m_id)).value
                    if dxl_addparam_result != 1:
                        print("[ID:%03d] groupSyncRead addparam failed" % (m_id))

            # Protocol 1.0 doesn't have sync read, so use bulk read instead
            else:
                d.group_num = dynamixel.groupBulkRead(d.port_num, d.protocol_version)
                d.gr_prespos = dynamixel.groupBulkRead(d.port_num, d.protocol_version, d.ctrl_table.PRESENT_POSITION,
                                                       d.ctrl_table.LEN_PRESENT_POSITION)
                for m_id in d.motor_id:
                    # Add parameter storage for each Dynamixel's present position value to the Bulkread storage
                    dxl_addparam_result = ctypes.c_ubyte(
                        dynamixel.groupBulkReadAddParam(d.group_num, m_id, d.ctrl_table.PRESENT_POSITION,
                                                        d.ctrl_table.LEN_PRESENT_POSITION)).value
                    if dxl_addparam_result != 1:
                        print("[ID:%03d] groupBulkRead addparam failed" % (m_id))


            print "gw_goalpos {}".format(d.gw_goalpos)
            print "gr_prespos {}".format(d.gr_prespos)

    def set_torque_enable(self,total_data):
        # for each device, if an id in the table matches, set torque
        # TODO: this should not use the total_data syntax, it should either use the implicit id format or stick with total data
        for d in self.device:
            ctrl_table_value = getattr(d.ctrl_table,"TORQUE_ENABLE")
            for m_id, val in total_data:
                if m_id in d.motor_id:
                    dynamixel.write1ByteTxRx(d.port_num, d.protocol_version, m_id, ctrl_table_value, val)
                    dxl_comm_result = dynamixel.getLastTxRxResult(d.port_num, d.protocol_version)
                    dxl_error = dynamixel.getLastRxPacketError(d.port_num, d.protocol_version)
                    if dxl_comm_result != COMM_SUCCESS:
                        print(dynamixel.getTxRxResult(d.protocol_version, dxl_comm_result))
                    elif dxl_error != 0:
                        print(dynamixel.getRxPacketError(d.protocol_version, dxl_error))

    def set_all_command_position(self, angles):
        for d in self.device:
            # print d.motor_id
            for m_id, a in zip(d.motor_id, angles[:len(d.motor_id)]):

                # convert from radians to actuator values
                res = d.motor[m_id]["resolution"]
                command = rad2pos(a,res)

                # Add parameter storage for each Dynamixel's goal position value to the Syncwrite storage
                dxl_addparam_result = ctypes.c_ubyte(
                    dynamixel.groupSyncWriteAddParam(d.gw_goalpos, m_id, command, LEN_GOAL_POSITION)).value
                
                if dxl_addparam_result != 1:
                    print("[ID:%03d] groupSyncWrite addparam failed" % (m_id))
                    quit()

            # Syncwrite goal position
            dynamixel.groupSyncWriteTxPacket(d.gw_goalpos)
            dxl_comm_result = dynamixel.getLastTxRxResult(d.port_num, d.protocol_version)
            if dxl_comm_result != COMM_SUCCESS:
                print(dynamixel.getTxRxResult(d.protocol_version, dxl_comm_result))

            # Clear syncwrite parameter storage
            dynamixel.groupSyncWriteClearParam(d.gw_goalpos)
            angles = angles[len(d.motor_id):]

    def get_all_current_position(self):
        pos_data = []
        for d in self.device:

            if d.protocol_version == 2:
                # Syncread present position
                dynamixel.groupSyncReadTxRxPacket(d.gr_prespos)
                dxl_comm_result = dynamixel.getLastTxRxResult(d.port_num, d.protocol_version)
                if dxl_comm_result != COMM_SUCCESS:
                    print(dynamixel.getTxRxResult(d.protocol_version, dxl_comm_result))
                    return None

                # Check if groupsyncread data of all dynamixels are available:
                for m_id in d.motor_id:
                    dxl_getdata_result = ctypes.c_ubyte(
                        dynamixel.groupSyncReadIsAvailable(d.gr_prespos, m_id,
                                                           d.ctrl_table.PRESENT_POSITION, LEN_PRESENT_POSITION)).value
                    if dxl_getdata_result != 1:
                        print("[ID:%03d] groupSyncRead getdata failed" % (m_id))
                        quit()

                        # Get present position value for (m_id)
                    data = dynamixel.groupSyncReadGetData(d.gr_prespos, m_id,
                                                          d.ctrl_table.PRESENT_POSITION,
                                                          LEN_PRESENT_POSITION)
                    res = d.motor[m_id]["resolution"]
                    pos_data.append(pos2rad(data, res))
            else:
                # Bulkread present position and moving status
                dynamixel.groupBulkReadTxRxPacket(d.group_num)
                dxl_comm_result = dynamixel.getLastTxRxResult(d.port_num, d.protocol_version)
                if dxl_comm_result != COMM_SUCCESS:
                    print(dynamixel.getTxRxResult(d.protocol_version, dxl_comm_result))

                for m_id in d.motor_id:
                    # Check if groupbulkread data of Dynamixel#1 is available
                    dxl_getdata_result = ctypes.c_ubyte(
                        dynamixel.groupBulkReadIsAvailable(d.group_num, m_id, d.ctrl_table.PRESENT_POSITION,
                                                           d.ctrl_table.LEN_PRESENT_POSITION)).value
                    if dxl_getdata_result != 1:
                        print("[ID:%03d] groupBulkRead getdata failed" % (m_id))
                        quit()
                        # Get Dynamixel#1 present position value
                    data = dynamixel.groupBulkReadGetData(d.group_num, m_id, d.ctrl_table.PRESENT_POSITION,
                                                                           d.ctrl_table.LEN_PRESENT_POSITION)
                    res = d.motor[m_id]["resolution"]
                    pos_data.append(pos2rad(data, res))

        return pos_data

def rad2pos(rad, resolution):
    return int(round(rad * resolution / (2 * 3.1415926)))


def pos2rad(pos, resolution):
    return pos * (2 * 3.1415926) / resolution

if __name__ == "__main__":
    # tests if things work on R arm manipulator
    motor_id = [9, 10, 11, 12, 13, 14, 15]
    Device = DxlDevice(motor_id)

    joint_commands = [0,0,0,0,0,0,0]
    enable = [1,1,1,1,1,1,1]

    try:
        Device.initialize()
        Device.set_command_position(motor_id, joint_commands)
        print Device.get_current_position(motor_id)
        time.sleep(2)
    finally:
        pass