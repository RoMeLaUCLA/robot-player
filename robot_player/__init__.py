#!usr/bin/env python
__author__ = "Min Sung Ahn, Daniel Sun"
__email__ = "aminsung@gmail.com, danielsun@ucla.edu"
__copyright__ = "Copyright 2016 RoMeLa"
__date__ = "September 1, 2017"

__version__ = "0.0.1"
__status__ = "Prototype"

from robot_player.motion_manager import MotionManager, player_arg_parser, from_player_angle_offset, to_player_angle_offset
from robot_player.vrep_interface import VrepInterface, VrepOptions
from robot_player.dxl_interface import DxlInterface, DxlOptions
