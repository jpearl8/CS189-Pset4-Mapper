import random
import math
from math import radians, degrees
import cv2
import numpy as np
from random import randint


def closest_num(my_arr, my_int):
    dif = 1000
    i = 0
    while (i != np.size(my_arr)):
        #print "i %d" % (i)
        current_dif = np.abs(my_arr[i] - my_int)
        if (current_dif == 0):
            #print "dif_x  is  0? %d where x is %d" % (current_dif, my_arr[i])
            close_num = my_arr[i]
            break
        elif (current_dif < dif):
            dif = current_dif
            #print "dif_x  is %d where x is %d" % (dif, my_arr[i])
            close_num = my_arr[i]
        i+=1
    return close_num

def closest_spot(my_big_arr, my_spot):
    my_arr_x = [idx[0] for idx in my_big_arr]
    my_arr_y = [idx[1] for idx in my_big_arr]

    x_spot = closest_num(my_arr_x, my_spot[0])
    y_spot = closest_num(my_arr_y, my_spot[1])

    return [x_spot, y_spot]


def dist(pos1, pos2):
    """
    Get cartesian distance between the (x, y) positions
    :param pos1: (x, y) position 1
    :param pos2: (x, y) position 2
    :return: Distance (float)
    """
    return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)


def orient(curr_pos, goal_pos):
    """
    Get necessary heading to reach the goal
    :param curr_pos: (x, y) current position of the robot
    :param goal_pos: (x, y) goal position to orient toward
    """
    return math.atan2(
        goal_pos[1] - curr_pos[1],
        goal_pos[0] - curr_pos[0])


def angle_compare(curr_angle, goal_angle):
    """
    Determine the difference between the angles, normalized from -pi to +pi
    :param curr_angle: current angle of the robot, in radians
    :param goal_angle: goal orientation for the robot, in radians
    """
    pi2 = 2 * math.pi
    # Normalize angle difference
    angle_diff = (curr_angle - goal_angle) % pi2
    # Force into range 0-2*pi
    angle_diff = (angle_diff + pi2) % pi2
    # Adjust to range of -pi to pi
    if (angle_diff > math.pi):
        angle_diff -= pi2
    return angle_diff


def sign(val):
    """
    Get the sign of direction to turn if >/< pi
    :param val: Number from 0 to 2*pi
    :return: 1 if val < pi (left turn), else -1 (right turn)
    """
    if val < 0:
        return -1
    elif val > 0:
        return 1
    return 0

def waypoint_navigation():
        """
        Navigate to a point given current orientation, orientation to destination 
        :return: Twist object of movement to make
        """
        # Movement command to send to the robot
        move_cmd = Twist()

        # Get angle difference of robot from destination (-pi to pi)
        angle_diff = angle_compare(self.dest_orientation, self.orientation)
        # Determine turn angle (proportional control with upper bound)
        # This also selects the direction that minimizes required turning
        prop_angle = abs(angle_diff) * ROT_K
        turn_angle = sign(angle_diff) * min(prop_angle, ROT_SPEED)

        if self.state == 'orient':
            # Orient to the destination (turning only) until with 5 degrees of goal
            move_cmd.angular.z = turn_angle
            if abs(angle_diff) < math.radians(5):
                self.state = 'approach'
        elif self.state == 'approach':
            # Move toward the destination (proportional control for linear and angular)
            # Robot can only move FORWARD (for safety), since distance to destination
            # is always positive, though the robot should turn around to return to the
            # goal if it overshoots
            move_cmd.angular.z = turn_angle
            destination_dist = dist(self.position, self.destination)
            move_cmd.linear.x = min(LIN_SPEED, destination_dist * LIN_K)
            if destination_dist < 0.05:
                # Consider destination reached if within 5 cm
                self.destination_reached = True
        # else will return empty/stop move_cmd

        return move_cmd


if __name__ == '__main__':

    # want to see the whole map when we print
    np.set_printoptions(threshold=np.inf)

    # map is initially completely unknown 
    fake_map = -np.ones((40,30))

    # fake simulation of certain blocks being set to free and occupied 
    for i in range(0, randint(10, 110)):
        x = randint(1, 39)
        y = randint(1, 29)

        obs_x = x + 2
        obs_y = y

        fake_map[x,y] = 0 
        fake_map[min(obs_x, 39), min(obs_y, 29)] = 1
    
    # create array of all positions that are 0 -- would need to have this update constantly as things are zerod out 
    free_map =  np.argwhere(fake_map == 0)

    # create array of all positions that are 1
    occ_map = np.argwhere(fake_map == 1)

    # select the one that is near to closest my position  -- chosen randomly
    my_pos = [randint(1, 39), randint(1, 29)]
    print "my_pos %s" % my_pos
   
    close_spot = closest_spot(free_map, my_pos)
    print "close_spot %s" % close_spot


    # go to that position --- veryy simple 
    self.state = 'orient'
    # Set the orientation to the destination
    self.dest_orientation = orient(self.position, self.destination)
    move_cmd = self.waypoint_navigation()
    self.cmd_vel.publish(move_cmd)

    self.rate.sleep()

     """ TODO:
        - think about how the list of free spots will be updated as the robot moves
        - when will the robot decide to move to a new spot?
        - instead of one closest spot, have list of closest spots and robot goes to them in that order? 
     """

    # # looping through lots of waypoints -- not really what we are doing 
    # if self.destination_reached:
    #     # If waypoint reached, go to the next one (and loop through waypoints)
    #     self.destination_reached = False
    #     self.waypoint_ind = (self.waypoint_ind + 1) % len(self.waypoints)
    #     self.destination = self.waypoints[self.waypoint_ind]
    #     self.state = 'orient'
    #     # Set the orientation to the destination
    #     self.dest_orientation = orient(self.position, self.destination)
    # move_cmd = self.waypoint_navigation()


    # set position to known 
    # delete that unknown from array 
        
    