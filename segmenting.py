"""	
Pset 4b 	
Makes basic map	with segmenting
"""	




import random	
import math	
from math import radians, degrees	
import cv2	
import numpy as np	
import rospy	
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

 FLOOR_HEIGHT = 100  # bottom pixels of img to ignore	
NUM_SEGMENTS = 5  # Number of segments to divide the depth image into	
world_map_ratio = 0.2 	

 def dist(pos1, pos2):	
    """	
    Get cartesian distance between the (x, y) positions	
    :param pos1: (x, y) position 1	
    :param pos2: (x, y) position 2	
    :return: Distance (float)	
    """	
    return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)	


 def centroid(contour):	
        """	
        Compute the (x,y) centroid position of the counter	
        :param contour: OpenCV contour	
        :return: Tuple of (x,y) centroid position	
        """	

         def centroid_x(c):	
            """	
            Get centroid x position	
            :param c: OpenCV contour	
            :return: x position or -1	
            """	
            M = cv2.moments(c)	
            if M['m00'] == 0:	
                return -1	
            return int(M['m10'] / M['m00'])	

         def centroid_y(c):	
            """	
            Get centroid y position	
            :param c: OpenCV contour	
            :return: y position or -1	
            """	
            M = cv2.moments(c)	
            if M['m00'] == 0:	
                return -1	
            return int(M['m01'] / M['m00'])	

         return centroid_x(contour), centroid_y(contour)	



 class Segmenting:	
    def __init__(self):	
        #INITIALIZER: 	
        # initializer for the maps 	
         # initialize MapDrawer object	
        self.mapObj = mp.MapDrawer(self.positionToMap)	

         # create blank array of negative ones to represent blank map 	

         self.map_size = (30,40)	
        self.my_map = -np.ones(self.map_size).astype(int)	


         self.depth_image =  np.zeros((480, 640))	
        # Localization determined from EKF	
        # Position is a geometry_msgs/Point object with (x,y,z) fields	
        self.position = [3, 3] # just for testing purposes, reset to None later 	
        # Orientation is the rotation in the floor plane from initial pose	
        self.orientation = None	

         # handle obstacles	
        self.obstacle_seen = [0, -1] # 0 for false, -1 for segment of obstacle	
        self.obstacle_depth = -1	
        self.seg_depth = [-1, -1, -1, -1, -1]	
        self.obstacle_pos = [0, 0]	
        self.obstacle = False	
        self.obstacle_side = 'right'	


         # speed and radians for turns set here	
        self.lin_speed = 0.2  # m/s	
        self.rot_speed = radians(90)	
        self.neg_rot = -radians(90)	

         self.crbump = False	
        self.lbump = False	

         self.lobstacle = False	
        self.robstacle = False	

         # section based on obstacles	
        self.seg_depth = [-1, -1, -1, -1, -1]	

         self.paused = False	
        self.position = None	
        self.orientation = None	

         # Initiliaze	
        rospy.init_node('Segmenting', anonymous=False)	

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
        (r, c) -> (x, y)	
        """	


         step_x = int(position[0]/world_map_ratio)	
        step_y = int(position[1]/world_map_ratio)	

         return (step_x + 2, step_y + 15)	


     def positionFromMap(self, position):	
        """	
        turn map positions back to EKF for display purposes	
        (x, y) -> (r c)	
        """	

         step_x = (position[0] - 2) * world_map_ratio	
        step_y = (position[1] - 15) * world_map_ratio	

         return (step_x, step_y)	

     def initializeMap(self):	
        print "initialize map"	
        # first map update, need to do twice because it doesn't show up nicely the first time .p	
        self.mapObj.UpdateMapDisplay(self.my_map, (0, 0))	
        self.mapObj.UpdateMapDisplay(self.my_map, (0, 0))	
        # show map for this amount of time 	
        time.sleep(0.001)	

     def updateMapFree(self, current_pos_map):	
        """	
        Takes a (x, y) as a parameter.	
        Calling UpdateMapDisplay on (r, c)	
        """	
#print "map free"	
        # update map with free position	
        current_pos = self.positionFromMap(current_pos_map)	

         # check that current pos in the map is within the bounds	
        # TODO: Should this be <=300 and 400 or 30 and 40?? It's in (x, y)	
        if (current_pos_map[0] <= 300 and current_pos_map[0] >= 0 and current_pos_map[1] <= 400 and current_pos_map[1] >= 0):	
            # if the current position is ok, set it to be free and update and show the map 	
            self.my_map[current_pos_map[0], current_pos_map[1]] = 0	
            self.my_map[current_pos_map[0], current_pos_map[1]-1] = 0	
            self.my_map[current_pos_map[0]-1, current_pos_map[1]-1] = 0	
            self.my_map[current_pos_map[0]-1, current_pos_map[1]] = 0	
            self.mapObj.UpdateMapDisplay(self.my_map, current_pos)	
            #print "current map pos: %d, %d" % (current_pos_map[0], current_pos_map[1])	
            time.sleep(0.0000001)   	

     def updateMapObstacle(self, segment):	
       # print "update map obstacle"	
        # position from (r,c) to (x, y)	
        (pos_x, pos_y) = self.positionToMap(self.position)	

         # obstacle coordinate calculation using depth	
        if (not(math.isnan(self.orientation)) and not(math.isnan(pos_x)) and not(math.isnan(pos_y))):	
            time.sleep(0.0000001) 	
            self.obstacle_pos[0] = int(pos_x + abs(self.seg_depth[segment])*np.cos(self.orientation + radians(60 - 30*segment)))	
            self.obstacle_pos[1] = int(pos_y + abs(self.seg_depth[segment])*np.sin(self.orientation + radians(60 - 30*segment)))	

             # loop through to free coordinates in front of obstacles	
            for x in range(0, int(abs(pos_x - self.obstacle_pos[0]))):	
                for y in range(0, int(abs(self.obstacle_pos[1] - pos_y))):	
                    (x1, y1) = (min(self.obstacle_pos[0], pos_x) + x, 	
                                min(self.obstacle_pos[1], pos_y) + y)	
                    self.updateMapFree((x1, y1))	
                    time.sleep(0.0000001) 	

             # final check before mapping obstacle (may not be necessary)	
            if (self.seg_depth[segment] != -1):	
                self.updateMapOccupied() 	
                time.sleep(0.0000001) 	
            else:	
                self.updateMapFree((self.obstacle_pos[0], self.obstacle_pos[1])) 	
                time.sleep(0.0000001) 	


     def updateMapOccupied(self):	
        """	
        self.obstacle_pos is in (x, y)	
        self.position is in (r, c)	
        """	
      #  print "obstacle loop"	

         # update map with position of obstacle and knowledge that that position will be occupied 	
        current_pos = self.position	
        current_orr = self.orientation	
        obstacle_orr = self.orientation # to be changed	
        current_pos_map = self.positionToMap(current_pos)	
        obstacle_pos_map = self.obstacle_pos	

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



     def closest_num(self, my_arr, my_int):	
        "find the number in array that is closes to a given number"	
        dif = 1000	
        i = 0	
        while (i != np.size(my_arr)):	
            current_dif = np.abs(my_arr[i] - my_int)	
            if (current_dif == 0):	
                close_num = my_arr[i]	
                break	
            elif (current_dif < dif):	
                dif = current_dif	
                close_num = my_arr[i]	
            i+=1	
        return close_num	

     def closest_spot(self, my_big_arr, my_spot):	
        "decide the closest spot for a given array and a given spot"	
        my_arr_x = [idx[0] for idx in my_big_arr]	
        my_arr_y = [idx[1] for idx in my_big_arr]	

         x_spot = self.closest_num(my_arr_x, my_spot[0])	
        y_spot = self.closest_num(my_arr_y, my_spot[1])	

         return [x_spot, y_spot]	

     def nextDest(self):	
        "decide where robot should  go next - frontier exploration"	
        free_map =  np.argwhere(self.my_map == 0)	
        position = self.position	
        close_spot = self.closest_spot(free_map, position)	

         return close_spot	

     def nextMove(self):	
        # twist object for this function only 	
        print "in next move"	
        move_cmd = Twist()	

         goal_pos = self.nextDest()	
        curr_pos = self.position	

         # set orientation to dest	
        dest_orient = math.atan2(goal_pos[1] - curr_pos[1], goal_pos[0] - curr_pos[0])	

         # navigate to this destination by getting the angle difference between current orientation and dest orientation 	
        # must be in range of 360 degrees or 2pi	
        pi2 = 2 * math.pi	
        angle_diff = (self.orientation - dest_orient) % pi2	

         # force into the range of 0 - 2pi	
        angle_diff = (angle_diff + pi2) % pi2	

         # Adjust to range of -pi to pi	
        if (angle_diff > math.pi):	
            angle_diff -= pi2	

         # choose sign for angle 	
        if angle_diff < 0:	
            return -angle_diff	

         # get the turn angle, direction for min turning - can add stuff to finnesse later 	
        turn_angle = angle_diff * min(angle_diff * 5, math.radians(30))	

         # orient to destination 	
        move_cmd.angular.z = turn_angle	

         destination_dist = dist(self.position, goal_pos)	

         # set movement speed tp destination	
        move_cmd.linear.x = min(0.3, destination_dist*0.2)	

         if destination_dist < 0.0005:	
            print "TOO CLOSE!"	
            move_cmd.linear.x = 0	

         return move_cmd	

     def wander(self):	
        """	
        Run until Ctrl+C pressed	
        :return: None	
        """	
        # add freeLoop function when ready	
        # Initialize by starting first side	
        self.pause = rospy.Time.now()	
        printed_position = False	

         move_cmd = Twist()	
        backwards = Twist()	
        turn = Twist()	

         turn.linear.x = 0	
        turn.angular.z = radians(180)	

         lobstacle = Twist()	
        robstacle = Twist()	
        bump = Twist()	

         backwards.linear.x = -self.lin_speed	
        backwards.angular.z = 0	

         bump.linear.x = 0	
        bump.angular.z = self.rot_speed	


         robstacle.linear.x = 0	
        robstacle.angular.z = radians(45)	

         lobstacle.linear.x = 0	
        lobstacle.angular.z = (-radians(45))	

         while not rospy.is_shutdown():	
            if (self.crbump | self.lbump):	
                rospy.sleep(1)	
                self.obstacle = True	

                 #need to make sure freeLoop works for bumps too!!!	
                for i in range (0, 3):	
                    self.cmd_vel.publish(backwards)	
                    self.rate.sleep()	
                rospy.sleep(1)	

                 if (self.crbump):	
                    bump.angular.z = self.rot_speed	
                else: 	
                    bump.angular.z = self.neg_rot	

                 for i in range(10):	
                    self.cmd_vel.publish(bump)	
                    self.rate.sleep()	
                rospy.sleep(1) 	
                self.crbump = False	
                self.lbump = False	
       # else:	
                #self.cmd_vel.publish(move_cmd)	
            # turning 90 in each direction and surveying	
           # for i in range(4):	
                #print "survey %d" % (i)	

                 #self.cmd_vel.publish(turn)	
                #self.rate.sleep()	


                 # for i in range(2):	
                #     self.cmd_vel.publish(move_cmd)	



             # while(self.robstacle):	
            #     self.freeLoop()                    	
            #     for i in range (0, 2):	
            #         self.cmd_vel.publish(robstacle)	
            #         self.rate.sleep()	
            #         rospy.sleep(.5)	
            #     self.robstacle = False	

             # while(self.lobstacle):	
            #     self.freeLoop()	
            #     for i in range (0, 2):	
            #         self.cmd_vel.publish(lobstacle)	
            #         self.rate.sleep()	
            #         rospy.sleep(.5)	
            #     self.lobstacle = False	

             else:	
                self.updateMapFree(self.positionToMap(self.position))	
                move_cmd.linear.x = 0	
            #     move_cmd.angular.z = 0	


             # # publish the velocity	
          #  print "hi"	
            self.cmd_vel.publish(move_cmd)	
            self.rate.sleep()	

             # # if this code works lol 	
            # # for i in range (0, 5):	
            # #     print "MOVING IN MAIN"	
            # #     move_cmd = self.nextMove()	
            # #     self.cmd_vel.publish(move_cmd)	
            # #     self.rate.sleep()	





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
        - Draws a bounding box around the largest object in the scene and returns	
        - Lets us know when obstacles have been seen	
        - Lets us know when to avoid obstacles	
        :param: Image described by an array	
        :return: Image with bounding box 	
        """	

         img = np.copy(img_in)	
        img_height, img_width = img.shape[:2] # (480, 640) 	
        NUM_SEGMENTS = 5 #segmenting robot vision by 5 (could be changed to 3 if lag/ not accurate)	


         # Get contours on each segment	
        for segment in range(NUM_SEGMENTS):	
            time.sleep(0.0000001)	
            seg_im = img[:-200, segment*(img_width/NUM_SEGMENTS):(segment+1)*(img_width/NUM_SEGMENTS)]	
            contours, _ = cv2.findContours(seg_im, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)	
            if len(contours) > 0:	
                # Find the largest contour	
                areas = [cv2.contourArea(c) for c in contours]	
                max_index = np.argmax(areas)	
                max_contour = contours[max_index]	
                new_obstacle_pos = centroid(max_contour)	
              #  print "bound called %d" % (segment)	
                time.sleep(0.0000001)	
                # returns obstacle depth that will allow obstacle to be mapped 	
                # sets depth of 0 to be -1 to avoid unintentional mapping	
                if new_obstacle_pos:	
                    self.seg_depth[segment] = (self.depth_image[new_obstacle_pos[0]-200][new_obstacle_pos[1]])/world_map_ratio 	
                    if (self.seg_depth[segment] == 0):	
                        self.seg_depth[segment] = -1	
                    #print "depth!"	
                    # self.seg_depth =  self.depth_image[new_obstacle_pos[0]][new_obstacle_pos[1]] 	

                 # show where largest obstacle is 	
                cv2.drawContours(img, max_contour, -1, color=(0, 255, 0), thickness=3)	

                 # Draw rectangle bounding box on image	
                x, y, w, h = cv2.boundingRect(max_contour)	

                 # only want to map obstacle if it is large enough jjj	
                if (w*h > 200 & segment != 2):	
                    self.updateMapObstacle(segment)	
                    self.obstacle_seen = [1, segment]	

                 # obstacle must be even larger or be right in front to get the state to be switched 	
                elif (w*h > 400 | ((w*h > 200) & (segment == 2))):	
                    self.updateMapObstacle(segment)	
                    self.state = 'avoid_obstacle'	
                    # Differentiate between left and right objects	
                    if (segment < 2): 	
                        self.obstacle_side = 'left'	
                    if (segment == 2):	
                        self.obstacle_side = 'center'	
                    else:	
                        self.obstacle_side = 'right'         	

         return img	

     def process_depth_image(self, data):	
        """ 	
        - Use bridge to convert to CV::Mat type. (i.e., convert image from ROS format to OpenCV format)	
        - Displays thresholded depth image 	
        - Calls bound_object function on depth image	
        :param: Data from depth camera	
        :return: None	
        """	
        try:	

             cv_image = self.bridge.imgmsg_to_cv2(data)	

             # create a mask to restrict the depth that can be seen and floor height	
            mask = cv2.inRange(cv_image, 0.1, 1)	
            #mask[:, :] = 0	
            im_mask = cv2.bitwise_and(cv_image, cv_image, mask=mask)	
            self.depth_image = im_mask	

             # bound the largest object per segment	
            dst2 = self.bound_object(mask)	


             # Normalize values to range between 0 and 1 for displaying	
            norm_img = im_mask	
            cv2.normalize(norm_img, norm_img, 0, 1, cv2.NORM_MINMAX)	

             # Displays thresholded depth image   	
            cv2.imshow('Depth Image', norm_img)    	
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
    robot = Segmenting()	
    robot.wander()	
    # except Exception, err:	
    #     rospy.loginfo("DepthScan node terminated.")	
    #     print err
