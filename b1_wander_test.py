"""
Pset 4a 
Makes basic map
"""


import rospy
import random
import math
from math import radians, degrees
import cv2
import numpy as np
from geometry_msgs.msg import Twist
import tf
from kobuki_msgs.msg import BumperEvent, CliffEvent, WheelDropEvent
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Quaternion, PointStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Empty

import map_util as mp
import numpy as np
import time





class B1_Wander_Test:
    def __init__(self):
        #INITIALIZER: 
        # initializer for the maps 
         # initialize MapDrawer object
        self.mapObj = mp.MapDrawer(self.positionToMap)

        # create blank array of negative ones to represent blank map 
        self.my_map = -np.ones((40,30))

        # Localization determined from EKF
        # Position is a geometry_msgs/Point object with (x,y,z) fields
        self.position = [3, 3] # just for testing purposes, reset to None later 
        # Orientation is the rotation in the floor plane from initial pose
        self.orientation = None

        # handle obstacles
        self.obstacle_pos = [0, 0]
        self.obstacle = False

        # speed and radians for turns set here
        self.lin_speed = 0.2  # m/s
        self.rot_speed = radians(90)
        self.neg_rot = -radians(90)


        self.crbump = False
        self.lbump = False

        self.lobstacle = False
        self.robstacle = False

        self.paused = False
        self.position = None
        self.orientation = None

        # Initiliaze
        rospy.init_node('Basic_Map', anonymous=False)

        # Tell user how to stop TurtleBot
        rospy.loginfo("To stop TurtleBot CTRL + C")
        # What function to call when you ctrl + c    
        rospy.on_shutdown(self.shutdown)


        
        # Create a publisher which can "talk" to TurtleBot wheels and tell it to move
        self.cmd_vel = rospy.Publisher('cmd_vel_mux/input/navi',Twist, queue_size=10)

        # Subscribe to queues for receiving sensory data
        rospy.Subscriber('mobile_base/events/bumper', BumperEvent, self.process_bump_sensing)

        # Use a CvBridge to convert ROS image type to CV Image (Mat)
        self.bridge = CvBridge()
        # Subscribe to depth topic
        rospy.Subscriber('/camera/depth/image', Image, self.process_depth_image, queue_size=1, buff_size=2 ** 24)

        
        # Subscribe to robot_pose_ekf for odometry/position information
        rospy.Subscriber('/robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, self.process_ekf)

        # Set up the odometry reset publisher (publishing Empty messages here will reset odom)
        reset_odom = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=1)
        # Reset odometry (these messages take about a second to get through)
        timer = rospy.Time.now()
        while rospy.Time.now() - timer < rospy.Duration(1) or self.position is None:
            reset_odom.publish(Empty())
        # TurtleBot will stop if we don't keep telling it to move.  How often should we tell it to move? 5 Hz
        self.rate = rospy.Rate(5)

    def positionToMap(self, position):
        """
        turn EKF position in meters into map position in coordinates
        """
        # ratio of world meters to map coordinates 
        world_map_ratio = 0.2

        step_x = int(position[0]/world_map_ratio)
        step_y = int(position[1]/world_map_ratio)
        
        return (step_x + 2, step_y + 15)

    def initializeMap(self):
        # first map update, need to do twice because it doesn't show up nicely the first time .p
        self.mapObj.UpdateMapDisplay(self.my_map, (0, 0))
        self.mapObj.UpdateMapDisplay(self.my_map, (0, 0))
      
        # show map for this amount of time 
        time.sleep(0.001)

    def updateMapFree(self):
        # update map with current position and knowlege that this position is free
        current_pos = self.position
        current_orr = self.orientation
        current_pos_map = self.positionToMap(current_pos)

        # check that current pos in the map is within the bounds
        if (current_pos_map[0] <= 300 and current_pos_map[0] >= 0 and current_pos_map[1] <= 400 and current_pos_map[1] >= 0):
                # if the current position is ok, set it to be free and update and show the map 
                self.my_map[current_pos_map[0], current_pos_map[1]] = 0
                self.my_map[current_pos_map[0], current_pos_map[1]-1] = 0
                self.my_map[current_pos_map[0]-1, current_pos_map[1]-1] = 0
                self.my_map[current_pos_map[0]-1, current_pos_map[1]] = 0
                self.mapObj.UpdateMapDisplay(self.my_map, current_pos)
                print "current map pos: %d, %d" % (current_pos_map[0], current_pos_map[1])
                time.sleep(0.0000001)            
        else:
            # if the current position is not ok, let it be known that the values are off, do not change the map array
            print "values are off! current map pos: %d, %d" % (current_pos_map[0], current_pos_map[1])

    def updateMapOccupied(self):
        # update map with position of obstacle and knowledge that that position will be occupied 
        print "float me"

        current_pos = self.position
        current_orr = self.orientation
        obstacle_pos = self.obstacle_pos
        obstacle_orr = self.orientation

        current_pos_map = self.positionToMap(current_pos)
        obstacle_pos_map = self.positionToMap(obstacle_pos)
        print "OBSTACLE MAP POS: %d, %d" % (obstacle_pos_map[0], obstacle_pos_map[1])

        # check that obstacle pos in the map is ok
        if (current_pos_map[0] <= 30 and current_pos_map[0] >= 0 and current_pos_map[1] <= 40 and current_pos_map[1] >= 0):
                # if the obstacle position is ok, set it to be occupied
                self.my_map[obstacle_pos_map[0], obstacle_pos_map[1]] = 1
                self.my_map[obstacle_pos_map[0]-1, obstacle_pos_map[1]] = 1
                self.my_map[obstacle_pos_map[0]-1, obstacle_pos_map[1]-1] = 1
                self.my_map[obstacle_pos_map[0], obstacle_pos_map[1]-1] = 1
                # show the map, but still relative to current position 
                self.mapObj.UpdateMapDisplay(self.my_map, current_pos)
                time.sleep(0.00000001)
        else:
            # if the current position is not ok, let it be known that the values are off, do not change the map array
            print "obstacle map pos: %d, %d" % (obstacle_pos_map[0], obstacle_pos_map[1])
      
    def wander(self):
        """
        Run until Ctrl+C pressed
        :return: None
        """
        #self.initializeMap()
        # Initialize by starting first side
        self.state = 'forward'
        self.state_change_time = rospy.Time.now()
        printed_position = False

        move_cmd = Twist()
        backwards = Twist()

        lobstacle = Twist()
        robstacle = Twist()
        cr = Twist()
        left = Twist()



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
            # if (self.state_change_time - rospy.time.now() >= 10):
            #     rospy.shutdown()
            # while (self.state_change_time - rospy.time.now() < 10):
            if (self.crbump | self.lbump):
                rospy.sleep(1)
                self.obstacle = True
                rospy.loginfo("RIGHT BUMP")
                if (not(math.isnan(self.orientation)) and not(math.isnan(self.position[0])) and not(math.isnan(self.position[1]))):
                    self.obstacle_pos[0] = int(float(self.position[0]) + .25*np.cos(float(self.orientation)))
                    self.obstacle_pos[1] = int(float(self.position[1]) + .25*np.sin(float(self.orientation)))
                    self.updateMapOccupied()
                for i in range (0, 3):
                    self.cmd_vel.publish(backwards)
                    self.rate.sleep()
                rospy.sleep(1)

                if self.crbump:
                        rospy.loginfo("CENTER RIGHT BUMP")
                        for i in range(10):
                            self.cmd_vel.publish(cr)
                            self.rate.sleep()
                        rospy.sleep(1) 
                        self.crbump = False
                if self.lbump:
                    rospy.loginfo("LEFT BUMP")
                    for i in range(10):
                        self.cmd_vel.publish(left)
                        self.rate.sleep()
                    rospy.sleep(1) 
                    self.lbump = False


            while(self.robstacle):
                print "check1"
                rospy.loginfo("RIGHT OBSTACLE")                        
                if (not(math.isnan(self.orientation)) and not(math.isnan(self.position[0])) and not(math.isnan(self.position[1]))):
                    self.obstacle_pos[0] = int(float(self.position[0]) + .4*np.cos(float(self.orientation)))
                    self.obstacle_pos[1] = int(float(self.position[1]) + .4*np.sin(float(self.orientation)))
                    self.updateMapOccupied()


                for i in range (0, 2):
                    self.cmd_vel.publish(robstacle)
                    self.rate.sleep()
                    rospy.sleep(.5)
                self.robstacle = False

            while(self.lobstacle):
                rospy.loginfo("LEFT OBSTACLE")
                if (not(math.isnan(self.orientation)) and not(math.isnan(self.position[0])) and not(math.isnan(self.position[1]))):
                    self.obstacle_pos[0] = int(float(self.position[0]) + .4*np.sin(float(self.orientation)))
                    self.obstacle_pos[1] = int(float(self.position[1]) + .4*np.cos(float(self.orientation)))
                    self.updateMapOccupied()

                for i in range (0, 2):
                    self.cmd_vel.publish(lobstacle)
                    self.rate.sleep()
                    rospy.sleep(.5)
                self.lobstacle = False

            else:
                self.updateMapFree()
                #rospy.loginfo("HERE")
                move_cmd.linear.x = self.lin_speed
                move_cmd.angular.z = 0






            # publish the velocity
            self.cmd_vel.publish(move_cmd)
            self.rate.sleep()


   
    def process_ekf(self, data):
        """
        Process a message from the robot_pose_ekf and save position & orientation to the parameters
        :param data: PoseWithCovarianceStamped from EKF
        """
        # Extract the relevant covariances (uncertainties).
        # Note that these are uncertainty on the robot VELOCITY, not position
        cov = np.reshape(np.array(data.pose.covariance), (6, 6))
        x_var = cov[0, 0]
        y_var = cov[1, 1]
        rot_var = cov[5, 5]
        # You can print these or integrate over time to get the total uncertainty
        
        # Save the position and orientation
        pos = data.pose.pose.position
        self.position = (pos.x, pos.y)
        orientation = data.pose.pose.orientation
        list_orientation = [orientation.x, orientation.y, orientation.z, orientation.w]
        self.orientation = tf.transformations.euler_from_quaternion(list_orientation)[-1]

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
                if (x < 150):
                    self.lobstacle = True
                else:
                    self.robstacle = True

            cv2.rectangle(img, (x, y), (x + w, y + h), color=(255, 255, 255), thickness=2)


        return img

    def process_depth_image(self, data):
        # Use bridge to convert to CV::Mat type. (i.e., convert image from ROS format to OpenCV format)
        # NOTE: SOME DATA WILL BE 'NaN'
        # and numbers correspond to distance to camera in meters
        # This imports as the default data encoding. For the ASUS Xtion cameras,
        # this is '32FC1' (single precision floating point [32F], single channel [C1])
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data)

            dst	= cv2.inRange(cv_image, 0.1, 0.6)	
            dst[400:, :] = 0
            dst[:, 0:200] = 0
            dst[:, 440:] = 0

            dst2 = self.bound_object(dst)




            # Display the thresholded depth image
            # Normalize values to range between 0 and 1 for displaying
            norm_img = cv_image
            cv2.normalize(norm_img, norm_img, 0, 1, cv2.NORM_MINMAX)

            # Displays thresholded depth image   
            #cv2.imshow('Depth Image', np.hstack((dst, dst2)))    
            #cv2.waitKey(3)
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
    

    def shutdown(self):
        """
        Pre-shutdown routine. Stops the robot before rospy.shutdown 
        :return: None
        """
        self.mapObj.SaveMap("saved_map.png", self.position)
        # Close CV Image windows
        cv2.destroyAllWindows()
        # stop turtlebot
        rospy.loginfo("Stop TurtleBot")
        # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
        # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(5)

if __name__ == '__main__':
    # try:
    robot = B1_Wander_Test()
    robot.wander()
    # except Exception, err:
    #     rospy.loginfo("DepthScan node terminated.")
    #     print err
