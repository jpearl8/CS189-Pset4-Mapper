"""
Pset 4a 
Makes basic map
"""


import rospy
import random
from math import radians
import cv2
import numpy as np
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent, CliffEvent, WheelDropEvent
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

"""
TESTING FILES:
include description of what it's testing, keep difference in files
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
 
    
  
3) Populate_map (parameters: coordinate, occupied vs. free boolean: 0 is free, 1 is occupied)
  -mark cell at coordinate that it's passed in
  -updating map based on marked cell/coordinates/parameters 
4) Pause (different then a shutdown function)
  -leaves final map on the screen
  -saves current map as png
"""

class Basic_Map:
    """
    This is a class for implementing and updating a basic map
    """
    def __init__(self):
      #INITIALIZER: 
        #TODO: needs all info about maps and such

      # speed and radians for turns set here
      self.lin_speed = 0.2  # m/s
      self.rot_speed = radians(90)
      self.neg_rot = -radians(90)

      self.crbump = False
      self.lbump = False

      self.lobstacle = False
      self.robstacle = False

      self.paused = False

      # Initiliaze
      rospy.init_node('Basic_Map', anonymous=False)

      # Tell user how to stop TurtleBot
      rospy.loginfo("To stop TurtleBot CTRL + C")
      # What function to call when you ctrl + c    
      rospy.on_shutdown(self.shutdown)

    
      # Create a publisher which can "talk" to TurtleBot wheels and tell it to move
      self.cmd_vel = rospy.Publisher('wanderer_velocity_smoother/raw_cmd_vel',Twist, queue_size=10)

      # Subscribe to queues for receiving sensory data
      rospy.Subscriber('mobile_base/events/bumper', BumperEvent, self.process_bump_sensing)

      # Use a CvBridge to convert ROS image type to CV Image (Mat)
      self.bridge = CvBridge()
      # Subscribe to depth topic
      rospy.Subscriber('/camera/depth/image', Image, self.process_depth_image, queue_size=1, buff_size=2 ** 24)

      # TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 5 Hz
      self.rate = rospy.Rate(5)

    def wander(self):
      """
      TODO: get location from obstacle that it bumped or sees
      Run until Ctrl+C pressed
      :return: None
      """

      move_cmd = Twist()
      backwards = Twist()

      lobstacle = Twist()
      robstacle = Twist()
      cr = Twist()
      left = Twist()

      backwards.linear.x = -self.lin_speed
      backwards.angular.z = 0

      backwards.linear.x = -self.lin_speed
      backwards.angular.z = 0

      cr.linear.x = 0
      cr.angular.z = self.rot_speed

      left.linear.x = 0
      left.angular.z = self.neg_rot

      robstacle.linear.x = 0
      robstacle.angular.z = radians(45)

      lobstacle.linear.x = 0
      lobstacle.angular.z = (-radians(45))

      while not rospy.is_shutdown():
        while not self.paused:
          if (self.crbump | self.lbump):
            rospy.sleep(1)
            # wherever the object is = occupied
            # populate_map(object_pos, 1)
            for i in range (0, 3):
              self.cmd_vel.publish(backwards)
              self.rate.sleep()
            rospy.sleep(1)
            
            if self.crbump:
                    for i in range(10):
                        self.cmd_vel.publish(cr)
                        self.rate.sleep()
                    rospy.sleep(1) 
                    self.crbump = False
                if self.lbump:
                    for i in range(10):
                        self.cmd_vel.publish(left)
                        self.rate.sleep()
                    rospy.sleep(1) 
                    self.lbump = False


          while(self.robstacle):
            # wherever the object is = occupied
            # populate_map(object_pos, 1)
            for i in range (0, 2):
              self.cmd_vel.publish(robstacle)
              self.rate.sleep()
            rospy.sleep(.5)
            self.robstacle = False

          while(self.lobstacle):
            # wherever the object is = occupied
            # populate_map(object_pos, 1)
            for i in range (0, 2):
              self.cmd_vel.publish(lobstacle)
              self.rate.sleep()
            rospy.sleep(.5)
            self.lobstacle = False

          else:  
            rospy.loginfo('({:.2f}, {:.2f})\t{:.1f} deg'.format(
                self.position[0], self.position[1], degrees(self.orientation)))
            current_pos = (self.position[0], self.position[1])
            # current_pos = free
            # populate_map(current_pos, 0)
            move_cmd.linear.x = self.lin_speed
            move_cmd.angular.z = 0
          
          self.cmd_vel.publish(move_cmd)
          self.rate.sleep()


"""  -while not paused:
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
"""

    def bound_object(self, img_in):
        """
        Draw a bounding box around the largest object in the scene
        :param img: RGB camera image
        :return: Image with bounding box
        """
        img = np.copy(img_in)

        # Get contours
        contours, _ = cv2.findContours(img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 0:
            # Find the largest contour
            areas = [cv2.contourArea(c) for c in contours]
            max_index = np.argmax(areas)
            max_contour = contours[max_index]

            # Draw rectangle bounding box on image
            # Differentiate between left and right objects
            x, y, w, h = cv2.boundingRect(max_contour)
            if (w*h > 200):
                if (x < 220):
                    self.lobstacle = True
                else:
                    self.robstacle = True

            cv2.rectangle(img, (x, y), (x + w, y + h), color=(255, 255, 255), thickness=2)

        return img

    def process_depth_image(self, data):
        try:
            # Use bridge to convert to CV::Mat type. (i.e., convert image from ROS format to OpenCV format)
            # NOTE: SOME DATA WILL BE 'NaN'
            # and numbers correspond to distance to camera in meters
            # This imports as the default data encoding. For the ASUS Xtion cameras,
            # this is '32FC1' (single precision floating point [32F], single channel [C1])
            cv_image = self.bridge.imgmsg_to_cv2(data)

            # if you turn enough times, (maybe five times, make it turn randomly)
            dst	= cv2.inRange(cv_image, 0.1, .7)	
            dst[400:, :] = 0
            dst[:, 0:200] = 0
            dst[:, 440:] = 0

            dst2 = self.bound_object(dst)


            # Display the thresholded depth image
            # Normalize values to range between 0 and 1 for displaying
            norm_img = cv_image
            cv2.normalize(norm_img, norm_img, 0, 1, cv2.NORM_MINMAX)

            # Displays thresholded depth image   
            cv2.imshow('Depth Image', np.hstack((dst, dst2)))    
            cv2.waitKey(3)
        except CvBridgeError, err:
            rospy.loginfo(err)

    
    def process_bump_sensing(self, data):
        """
        If bump data is received, process the data
        data.bumper: LEFT (0), CENTER (1), RIGHT (2)
        data.state: RELEASED (0), PRESSED (1)
        :param data: Raw message data from bump sensor 
        :return: None
        """
                
        # Differentiating between types of bumper interactions:
        # logs different messages based on how object hits and reacts differently

        if (data.state == BumperEvent.PRESSED):
            if (data.bumper == BumperEvent.CENTER):
                if (data.bumper == BumperEvent.RIGHT & data.bumper == BumperEvent.LEFT | data.bumper == BumperEvent.RIGHT):
                    self.lbump = False
                    self.crbump = True
                    rospy.loginfo("Bumper Event: all 3")
                elif (data.bumper == BumperEvent.LEFT):
                    self.lbump = True
                    self.crbump = False
                    rospy.loginfo("Bumper Event: left center")
                else:
                    self.lbump = False
                    self.crbump = True
                    rospy.loginfo("Bumper Event: center")
            elif (data.bumper == BumperEvent.LEFT):
                self.lbump = True
                self.crbump = False
                rospy.loginfo("Bumper Event: left")
            elif (data.bumper == BumperEvent.RIGHT):
                self.crbump = True
                self.lbump = False  
    
    def pause(self):
      #TODO
      return "not done"

    def shutdown(self):
        """
        Pre-shutdown routine. Stops the robot before rospy.shutdown 
        :return: None
        """
        # Close CV Image windows
        cv2.destroyAllWindows()
        # stop turtlebot
        rospy.loginfo("Stop TurtleBot")
        # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
        # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(5)

    if __name__ == '__main__':
    try:
        robot = Basic_Map()
        robot.wander()
    except Exception, err:
        rospy.loginfo("DepthScan node terminated.")
        print err