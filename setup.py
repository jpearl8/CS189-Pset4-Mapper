import map_util as mp
import numpy as np
import math
import time

# modules i dont have :(
# from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Quaternion, PointStamped
# import tf
# from std_msgs.msg import Empty
# import sys
# import traceback
# import rospy
# from geometry_msgs.msg import Twist
# from kobuki_msgs.msg import BumperEvent


class MapMaker:
    "handle all things map"

    def __init__(self):

        # Localization determined from EKF
        # Position is a geometry_msgs/Point object with (x,y,z) fields
        self.position = None
        # Orientation is the rotation in the floor plane from initial pose
        self.orientation = None

    # --------- roscore stuff -- comment out as needed -------------------------
        # # Initialize
        # rospy.init_node('MapMaker', 'time', anonymous=False)
 
        # # Subscribe to robot_pose_ekf for odometry/position information
        # rospy.Subscriber('/robot_pose_ekf/odom_combined', PoseWithCovarianceStamped, self.process_ekf)

        # # Set up the odometry reset publisher (publishing Empty messages here will reset odom)
        # reset_odom = rospy.Publisher('/mobile_base/commands/reset_odometry', Empty, queue_size=1)
        # # Reset odometry (these messages take about a second to get through)
        # timer = rospy.Time.now()
        # while rospy.Time.now() - timer < rospy.Duration(1) or self.position is None:
        #     reset_odom.publish(Empty())

        # # 5 HZ
        # self.rate = rospy.Rate(5)

    # def process_EKF(self, data):
    #     """
    #     Process a message from the robot_pose_ekf and save position & orientation to the parameters
    #     :param data: PoseWithCovarianceStamped from EKF
    #     """
    #     # Save the position
    #     pos = data.pose.pose.position
    #     self.position = (pos.x, pos.y)

    #     # Save the orientation 
    #     orientation = data.pose.pose.orientation
    #     list_orientation = [orientation.x, orientation.y, orientation.z, orientation.w]
    #     self.orientation = tf.transformations.euler_from_quaternion(list_orientation)[-1]
        

    
    def positionToMap(self, position):
        """
        turn EKF position in meters into map position in coordinates
        """
        # ratio of world meters to map coordinates 
        world_map_ratio = 0.5

        # convert meters to map coorinates -- need to edit
        x = position[0]/world_map_ratio
        y = position[1]/world_map_ratio
        return (int(x), int(y))

    def run(self):
        """
        initialize and update map
        -1 = unknown, 0 = empty, 1 = occupied
        """

        # initialize MapDrawer object
        mapObj = mp.MapDrawer(self.positionToMap)

        # create blank array of negative ones to represent blank map 
        my_map = -np.ones((30,40))

        # first map update 
        mapObj.UpdateMapDisplay(my_map, (0, 0))
        mapObj.UpdateMapDisplay(my_map, (0, 0))
        # show map for this amount of time 
        time.sleep(0.5)

        # next map update - zeroing out four corners of the map relative to current position
        start_pos = (0, 0)
        start_pos_map = self.positionToMap(start_pos)
        print start_pos_map


        my_map[start_pos_map[0], start_pos_map[1]] = 0
        my_map[start_pos_map[0]-1, start_pos_map[1]] = 0
        my_map[start_pos_map[0]-1, start_pos_map[1]-1] = 0
        my_map[start_pos_map[0], start_pos_map[1]-1] = 0
        #print my_map
        mapObj.UpdateMapDisplay(my_map, start_pos)
        time.sleep(0.5)

        # update random spot with this to be occupied just to see, still relative to start_pos
        my_map[20, 30] = 1
        my_map[21, 31] = 1
        mapObj.UpdateMapDisplay(my_map, start_pos)
        time.sleep(10)


        # update map with current position and knowlege that this position is free



if __name__ == '__main__':
    make_a_map = MapMaker()
    make_a_map.run()