"""
Pset 4a 
Makes basic map
"""

import cv2
import rospy
import numpy as np
import math
from geometry_msgs.msg import Twist
from math import radians
from kobuki_msgs.msg import BumperEvent, Sound
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
