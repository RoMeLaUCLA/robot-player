from contextlib import contextmanager
import unittest
from robot_player import MotionManager, VrepOptions

@contextmanager
def motion_manager():
    """
    creates a context manager so that we can test the MotionManger using the normal 'with' syntax. this allows us to test the __enter__ and __exit__ methods.

    :return:
    """
    yield MotionManager(motor_ids=[1, 2], dt=.01, options=VrepOptions(joint_prefix="joint"))

class TestVrepOptions(unittest.TestCase):

    def run(self, result=None):
        with motion_manager() as mm:
            self.mm = mm
            super(TestVrepOptions, self).run(result)

    def test_init(self):
        self.assertEqual(self.mm.sim, True)


if __name__ == '__main__':
    unittest.main()
