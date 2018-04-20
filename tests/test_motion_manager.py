from contextlib import contextmanager
import unittest
from robot_player import MotionManager, VrepOptions

@contextmanager
def motion_manager():
    """
    creates a context manager so that we can test the MotionManger using the normal 'with' syntax. this allows us to test the __enter__ and __exit__ methods.

    :return:
    """
    # TODO: figure out how to independently test vrep and dxl in the same test file
    yield MotionManager(motor_ids=[1, 2], dt=.01, options=VrepOptions())

class TestVrepOptions(unittest.TestCase):

    def run(self, result=None):
        """
        overrides TestCase.run() in order to add the MotionManager object as a class attribute to the TestCase, then calls the parent run method.

        :param result:
        :return:
        """
        with motion_manager() as mm:
            self.mm = mm
            super(TestVrepOptions, self).run(result)

    def test_init(self):

        # using VrepOptions
        self.assertEqual(self.mm.sim, True)
        self.assertEqual(self.mm.dxl, False)

if __name__ == '__main__':
    unittest.main()
