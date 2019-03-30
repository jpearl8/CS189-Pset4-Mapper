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


"""
1) Map initializer
  -following documentation, setting up map array
  -translate EKF to map position
2) Wander function
  -while not paused:
    -while no obstacle:
      current_pos = free
      populate map with current position being free
      move forward
    -if obstacle:
      get position of obstacle (?from depth camera, coordinates of what it is avoiding)
      mark that position as occupied in map
      turn (in some direction, some amount)
      {basic:
        if still occupied, keep turning
        don't update map with additional turns }
      {start off basic, add later:
        if still occupied, update map and keep turning }
   -if paused:
      call pause function
    
  
3) Populate_map (parameters: coordinate, occupied vs. free boolean: 0 is free, 1 is occupied)
  -mark cell at coordinate that it's passed in
  -updating map based on marked cell/coordinates/parameters 
4) Pause (different then a shutdown function)
  -leaves final map on the screen
  -saves current map as png
"""
