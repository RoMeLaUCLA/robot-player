from contextlib import contextmanager
import unittest
from robot_player import MotionManager, VrepOptions, DxlOptions

@contextmanager
def motion_manager(opts, ids=[1, 2]):
    """
    creates a context manager so that we can test the MotionManger using the normal 'with' syntax. this allows us to test the __enter__ and __exit__ methods.

    :param opts: options object to use in the MotionManager
    :param ids: default motor ids used for vrep options
    :return:
    """
    yield MotionManager(motor_ids=ids, dt=.01, options=opts)

class TestVrepOptions(unittest.TestCase):

    def run(self, result=None):
        """
        overrides TestCase.run() in order to add the MotionManager object as a class attribute to the TestCase, then calls the parent run method.

        :param result: part of the parent class. just a default argument
        :return:
        """
        with motion_manager(VrepOptions()) as mm:
            self.mm = mm
            super(TestVrepOptions, self).run(result)

    def test_init(self):
        self.assertEqual(self.mm.sim, True)
        self.assertEqual(self.mm.dxl, False)
        self.assertEqual(self.mm.player, 'vrep')

class TestDxlOptions(unittest.TestCase):

    def run(self, result=None):
        """
        overrides TestCase.run() in order to add the MotionManager object as a class attribute to the TestCase, then calls the parent run method.

        :param result: part of the parent class. just a default argument
        :return:
        """
        motor_ids = [(1, 2), (3, 4)]
        with motion_manager(DxlOptions(motor_ids, motor_types=['MX106', 'MX106'], ports=['/dev/ttyUSB2', '/dev/ttyUSB0'], baudrate=3000000, protocol_version=2), motor_ids) as mm:
            self.mm = mm
            super(TestDxlOptions, self).run(result)

    def test_init(self):
        self.assertEqual(self.mm.sim, False)
        self.assertEqual(self.mm.dxl, True)
        self.assertEqual(self.mm.player, 'dxl')


if __name__ == '__main__':
    unittest.main()
