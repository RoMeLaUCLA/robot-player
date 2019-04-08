from time import sleep
from math import pi
import numpy as np
import platform
from robot_player import MotionManager, DxlOptions
from robot_player.dxl.dxl_control_table import DXLPRO, MX106, MX106_P1, MX28, MX28_P1, XSERIES

if platform.system() == 'Windows':
    ports = ['COM15']
else:
    ports = ['/dev/ttyUSB0']

# choose motor type
dxl_type_check = 0
while dxl_type_check !='y':
    dxl_num = int(input("Enter a number to choose dynamixel type. 1 for MX-106, 2 for DXLPRO."))
    if dxl_num == 1:
        dxl_str = "MX106"
        dxl_type = MX106

    elif dxl_num == 2:
        dxl_str = "DXLPRO"
        dxl_type = DXLPRO
    else:
        print("Please a number between 1 and 2.")
        exit(1)
    print('The type of dynamixel chosen is {}.'.format(dxl_str))
    dxl_type_check = input('Is that correct?(y/n)')
    if dxl_type_check != 'y':
        print('Try again.')


ids = [1, 2]
motor_ids = [[1, 2]]
dopts = DxlOptions(motor_ids,
                   motor_types=[dxl_str],
                   ports=ports,
                   baudrate=3000000,
                   protocol_version=2)

with MotionManager(ids, dt=.005, options=dopts) as mm:
    mm.initialize()
    di = mm.device
    # Read data

    # Checks
    address = dxl_type.ID# motor id
    print(di._read_data(ids, address, 1))
    assert ([1, 2] == di._read_data(ids, address, 1))
    # check to make sure that data gets read back in whatever order the ids were passed in.
    assert ([2, 1] == di._read_data([2, 1], address, 1))
    address = dxl_type.MODEL_NUMBER  # model number
    assert (di._read_data(ids, address, 2) == [54024, 54024]) # dynamixel pro model number

    # turn torque off before writing to EEPROM
    address = dxl_type.TORQUE_ENABLE
    di._write_data(ids, address, [0] * len(ids), 1)

    # write homing offset
    address = dxl_type.HOMING_OFFSET  # homing offset
    di._write_data(ids, address, [0] * len(ids), 4)
    print(di._read_data(ids, address, 4))
    assert (di._read_data(ids, address, 4) == [0, 0])

    # Set external ports mode to analog input
    if dxl_type == DXLPRO:
        print("reading from sensor")
        address = dxl_type.EXTERNAL_PORT_MODE_1
        for i in range(4):
            di._write_data(ids, address, [0] * len(ids), 1)
            address += 1

        # Read external data
        ft_data_raw = {1:[],2:[]} # initialize list
        address = DXLPRO.EXTERNAL_PORT_DATA_1
        for i in range(0,8,2):
            port_data = di._read_data(ids, address+i, 2)
            print(port_data)
            ft_data_raw[1].append(port_data[0])
            ft_data_raw[2].append(port_data[1])
        # ft_data = [0] * 6 # initialize list
        # this is if motor 1 is setup at the off port and 2 is at direct port
        print('Sensor data: {} {}.'.format(ft_data_raw[1], ft_data_raw[2]))

