# unittests for dxl_interface

from robot_player import DxlInterface, DxlOptions, VrepOptions, VrepInterface, MotionManager
import unittest

class dxl_interface_test(unittest.TestCase):
    def test_open_close(self):
        # tests that there are no problems with opening ports twice.
        # linux test, windows needs to change ports to COMXXX
        ids = [1, 2, 3, 4]
        motor_ids = [(1, 2), (3, 4)]
        dopts = DxlOptions(motor_ids,
                           motor_types=['MX106', 'MX106'],
                           ports=['/dev/ttyUSB0', '/dev/ttyUSB1'],
                           baudrate=3000000,
                           protocol_version=2)
        with MotionManager(motor_ids=ids, dt=.016, options=dopts) as mm:
            mm.initialize()

        with MotionManager(motor_ids=ids, dt=.016, options=dopts) as mm:
            mm.initialize()

    def test_bad_ctrl_table(self):
        motor_ids = [5, 6]
        dt = .01
        self.assertRaises(ValueError, DxlOptions, motor_ids=[motor_ids],
                                                  motor_types=['DXL_PRO'], # if there's an incorrectly named control table type, it gets caught
                                                  ports=['/dev/ttyUSB0'],
                                                  baudrate=3000000,
                                                  protocol_version=2)

class motion_manager_test(unittest.TestCase):
    def test_check_command_and_ids_are_same_length(self):
        print("Check that command and ids are same length")
        ids = [1, 2, 3, 4]
        vopts = VrepOptions()
        with MotionManager(motor_ids=ids, dt=.016, options=vopts) as mm:
            mm.initialize()

            command_list = [mm.set_goal_position, mm.set_goal_velocity, mm.set_goal_effort]
            for command in command_list:
                self.assertRaises( ValueError, command, ids=[1,3],commands=[1,1,1]) # ids and commands are not the same length.


if __name__ == '__main__':
    unittest.main()