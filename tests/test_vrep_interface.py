import unittest
from robot_player import VrepOptions

class TestVrepOptions(unittest.TestCase):

    def test_init(self):
        # test the default arguments
        vopts = VrepOptions()
        self.assertEqual(vopts.joint_prefix, None)
        self.assertEqual(vopts.gyroscope, False)
        self.assertEqual(vopts.accelerometer, False)
        self.assertEqual(vopts.ft_sensor_names, None)
        self.assertEqual(vopts.opmode, None)


if __name__ == '__main__':
    unittest.main()
