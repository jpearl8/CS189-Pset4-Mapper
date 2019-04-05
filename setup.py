import map_util as mp
import numpy as np
import math
import time

#modules i dont have :(
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Quaternion, PointStamped
import tf
from std_msgs.msg import Empty
import sys
import traceback
import rospy
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent


class MapMaker:
    "handle all things map"

    def __init__(self):

        # initialize MapDrawer object
        self.mapObj = mp.MapDrawer(self.positionToMap)

        # create blank array of negative ones to represent blank map 
        self.my_map = -np.ones((30,40))

        # Localization determined from EKF
        # Position is a geometry_msgs/Point object with (x,y,z) fields
        self.position = (3, 3) # just for testing purposes, reset to None later 
        # Orientation is the rotation in the floor plane from initial pose
        self.orientation = None

        # handle obstacles
        self.obstacle_pos = None
        self.obstacle = False

    #--------- roscore stuff -- comment out as needed -------------------------
        # Initialize
        rospy.init_node('MapMaker', 'time', anonymous=False)
 
        # Subscribe to robot_pose_ekf for odometry/position information
        rospy.Subscriber('/robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, self.process_ekf)

        # Set up the odometry reset publisher (publishing Empty messages here will reset odom)
        reset_odom = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=1)
        # Reset odometry (these messages take about a second to get through)
        timer = rospy.Time.now()
        while rospy.Time.now() - timer < rospy.Duration(1) or self.position is None:
            reset_odom.publish(Empty())

        #stuff for wandering - subscribing to velocity publisher

        #Create a publisher which can "talk" to TurtleBot wheels and tell it to move
        self.cmd_vel = rospy.Publisher('wanderer_velocity_smoother/raw_cmd_vel',Twist, queue_size=10)

        #Tell user how to stop TurtleBot
        rospy.loginfo("To stop TurtleBot CTRL + C")
        #What function to call when you ctrl + c    
        rospy.on_shutdown(self.shutdown)

        # 5 HZ
        self.rate = rospy.Rate(5)

    def process_ekf(self, data):
        """
        Process a message from the robot_pose_ekf and save position & orientation to the parameters
        :param data: PoseWithCovarianceStamped from EKF
        """
        # Save the position
        pos = data.pose.pose.position
        self.position = (pos.x, pos.y)

        # Save the orientation 
        orientation = data.pose.pose.orientation
        list_orientation = [orientation.x, orientation.y, orientation.z, orientation.w]
        self.orientation = tf.transformations.euler_from_quaternion(list_orientation)[-1]

    def positionToMap(self, position):
        """
        turn EKF position in meters into map position in coordinates
        """
        # ratio of world meters to map coordinates 
        world_map_ratio = 0.2

        # rang of meters for robot to travel in x and y direction (ie dimensions of that box), need to be condensed into 30, 40 map 
        # position relative to 0,0 - so in steps? 
        # convert meters to map coorinates -- need to edit -- commented out the right hand assertion 
        step_x = int(position[0]/world_map_ratio)
        step_y = int(position[1]/world_map_ratio)
        if (position[0] != 0 and position != 1):
            step_x = step_x + 0
            step_y = step_y + 0
        
        return (step_x , step_y)

    def initializeMap(self):

        # first map update, need to do twice because it doesn't show up nicely the first time .p
        self.mapObj.UpdateMapDisplay(self.my_map, (0, 0))
        self.mapObj.UpdateMapDisplay(self.my_map, (0, 0))
        # show map for this amount of time 
        time.sleep(0.25)

        # next map update - zeroing out four corners of the map relative to current position -- not nessecary will change 
        start_pos = (0, 0)
        start_pos_map = self.positionToMap(start_pos)
        print start_pos_map
        self.my_map[start_pos_map[0], start_pos_map[1]] = 0
        self.my_map[start_pos_map[0]-1, start_pos_map[1]] = 0
        self.my_map[start_pos_map[0]-1, start_pos_map[1]-1] = 0
        self.my_map[start_pos_map[0], start_pos_map[1]-1] = 0
        #print my_map
        self.mapObj.UpdateMapDisplay(self.my_map, start_pos)
        time.sleep(0.25)

      
    def updateMapFree(self):
        # update map with current position and knowlege that this position is free
        current_pos = self.position
        current_pos_map = self.positionToMap(current_pos)

        # check that current pos in the map is within the bounds
        if (current_pos_map[0] <= 300 and current_pos_map[0] >= 0 and current_pos_map[1] <= 400 and current_pos_map[1] >= 0):
                # if the current position is ok, set it to be free and update and show the map 
                self.my_map[current_pos_map[0], current_pos_map[1]] = 0
                self.mapObj.UpdateMapDisplay(self.my_map, current_pos)
                print "map updated"
                time.sleep(0.5)            
        else:
            # if the current position is not ok, let it be known that the values are off, do not change the map array
            print "values are off! current map pos: %d, %d" % (current_pos_map[0], current_pos_map[1])

    def updateMapOccupied(self):
        # update map with position of obstacle and knowledge that that position will be occupied 
        obstacle_pos = self.obstacle_pos
        obstacle_pos_map = self.positionToMap(obstacle_pos)

        # check that obstacle pos in the map is ok
        if (current_pos_map[0] <= 30 and current_pos_map[0] >= 0 and current_pos_map[1] <= 40 and current_pos_map[1] >= 0):
                # if the obstacle position is ok, set it to be occupied
                self.my_map[current_pos_map[0], obstacle_pos_map[1]] = 1
                # show the map, but still relative to current position 
                self.mapObj.UpdateMapDisplay(self.my_map, current_pos)
                time.sleep(3)
        else:
            # if the current position is not ok, let it be known that the values are off, do not change the map array
            print "values are off! obstacle map pos: %d, %d" % (obstacle_pos_map[0], obstacle_pos_map[1])

    def shutdown(self):
        """
        Pre-shutdown routine. Stops the robot before rospy.shutdown 
        :return: None
        """
        # stop turtlebot
        rospy.loginfo("Stop Drawing Squares!")
        # publish a zeroed out Twist object
        self.cmd_vel.publish(Twist())
        # sleep before final shutdown
        rospy.sleep(2)



    def run(self):
        """
        for testing purposes - need to intergrate b1_wander_test.py
        initialize and update map
        -1 = unknown, 0 = empty, 1 = occupied
        """
        # initialize map 
        self.initializeMap()
        #initialize very basic wandering 
        move_cmd = Twist()
        move_cmd.linear.x = 2
        move_cmd.angular.z = 0.3

        
        rospy.loginfo('%d, %d'.format(
                    self.position[0], self.position[1]))

        #indent and uncomment for actual testing on robot 

        while not rospy.is_shutdown():
            #if an obstacle is seen, this boolean should become true and map is updated accordingly
	        #if (self.obstacle == True):
	            #self.updateMapOccupied()

	        

	        # actually move  
	        for i in range(0, 3):
	        	self.cmd_vel.publish(move_cmd)
	        	self.rate.sleep()
	        	rospy.sleep(0.5)
	        	print i
	        # avoid jerkiness by sleeping with each interval
	        
	        # update the map with every movement 
	        self.updateMapFree()
	        


if __name__ == '__main__':
    make_a_map = MapMaker()
    make_a_map.run()