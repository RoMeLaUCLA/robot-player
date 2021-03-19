# Daniel Sun 18 Mar 2021 
# pebl_bionic

import unittest
import logging

import robot_player.dxl.dxl_control_table as table
from robot_player import dxl_interface as dxl_int
from robot_player.dxl.dxl_control_table import DXLPRO
from robot_player.motion_manager import MotionManager

from pebl import settings

logging.basicConfig(level=logging.DEBUG)

class TestDXLPort(unittest.TestCase):
    def test_read_and_set_data(self):
        pL = dxl_int.DxlPort([1, 2, 3, 4, 5, 6],
                            'DXLPRO',
                            '/dev/serial/by-id/usb-FTDI_USB-COM485_Plus4_FTVTM57P-if02-port0',
                            baudrate=3000000,
                            protocol_version=2)
        pR = dxl_int.DxlPort([7,8,9,10,11,12],
                            'DXLPRO',
                            '/dev/serial/by-id/usb-FTDI_USB-COM485_Plus4_FTVTM57P-if03-port0',
                            baudrate=3000000,
                            protocol_version=2)

        with pL, pR:
            pL.open()
            pR.open()
            print(pL.read_data(DXLPRO.POSITION_P_GAIN, pL.motor_id, 2))
            print(pL.read_data(DXLPRO.ID, pL.motor_id, 1))
            print(pL.read_data(DXLPRO.PRESENT_POSITION, pL.motor_id, 4))

            # should hear humming after this
            pL.set_data(DXLPRO.TORQUE_ENABLE, pL.motor_id, [1, 1, 1, 1, 1, 1], 2)
            pR.set_data(DXLPRO.TORQUE_ENABLE, pL.motor_id, [1, 1, 1, 1, 1, 1], 2)

            pL.setup_control_table()
            pR.setup_control_table()
            pL.setup_group_sync_read('PRESENT_POSITION')
            pR.setup_group_sync_read('PRESENT_POSITION')
            print(pL.get_present_position([1,2,3,4,5,6]))
            print(pL.get_present_position([1,3,5]))
            print(pL.get_present_position([2,4,6]))
            print(pR.get_present_position([7,8,9,10,11,12]))


class TestDXLInterface(unittest.TestCase):
    def test_multiport_creation(self):
        # options = settings.dxl_options
        baudrate = 3000000
        dxl_ports = dxl_int.DxlPort.create_multiple_ports(
            motor_types=['DXLPRO', 'DXLPRO'],
            motor_ids=
            [[1, 2, 3, 4, 5, 6, ],
             [7, 8, 9, 10, 11, 12]],
            ports=[
                '/dev/serial/by-id/usb-FTDI_USB-COM485_Plus4_FTVTM57P-if02-port0',
                '/dev/serial/by-id/usb-FTDI_USB-COM485_Plus4_FTVTM57P-if03-port0'],
            baudrate=baudrate,
        )

        for p in dxl_ports:
            print(p)
            p.open()
            p.setup_control_table()

        dxl_ports[0].read_data(table.DXLPRO.POSITION_P_GAIN,
                               [1, 2, 3, 4, 5, 6], 2)

        di = dxl_int.DxlInterface(baudrate=baudrate,
                                  dxl_ports=dxl_ports)

        ids = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]

        data = di._read_data(ids, table.DXLPRO.VELOCITY_I_GAIN, 2)
        print(data)

        data = di._read_data(ids, table.DXLPRO.VELOCITY_P_GAIN, 2)
        print(data)

        data = di._read_data(ids, table.DXLPRO.POSITION_P_GAIN, 2)
        print(data)

    def test_interface_creation(self):
        baudrate = 3000000
        dxl_ports = dxl_int.DxlPort.create_multiple_ports(
            motor_types=['DXLPRO', 'DXLPRO'],
            motor_ids=
            [[1, 2, 3, 4, 5, 6, ],
             [7, 8, 9, 10, 11, 12]],
            ports=[
                '/dev/serial/by-id/usb-FTDI_USB-COM485_Plus4_FTVTM57P-if02-port0',
                '/dev/serial/by-id/usb-FTDI_USB-COM485_Plus4_FTVTM57P-if03-port0'],
            baudrate=baudrate,
        )
        di = dxl_int.DxlInterface(baudrate, dxl_ports)


class TestMotionManager(unittest.TestCase):
    def test_initialize_multiport(self):
        ids = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]
        mm = MotionManager(ids, dt=0.005, options=settings.dxl_options)
        with mm:
            pass
