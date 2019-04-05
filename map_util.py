"""
Package for drawing the generated map of the world for Pset 4 of CS 189.

2019-03: Created by Mark Petersen
"""

from math import *
import numpy as np
import cv2

class MapDrawer:
    """
    A class for incrementally updating the displayed image of the generated map,
    and saving the image to file.  The map must be an occupancy grid of shape
    (30, 40).
    """

    def __init__(self, positionToMap):
        """
        Creates a new MapDrawer object where `positionToMap`is a function that
        takes in a world position (in meters) and outputs the corresponding map
        coordinates. `positionToMap` must define a right-handed coordinate
        system.
        """
        self.map_size = (40,30)
        self.draw_scale = 16
        self.map = -np.ones(self.map_size).astype(int)
        self.drawn_map = np.zeros((640,480,3))
        self.map_colors = [[0, 0, 0], [1, 1, 1], [1, 0, 0]]
        self.positionToMap = positionToMap

        origin = positionToMap((0, 0))
        x_axis = positionToMap((1, 0))
        self.axis_orientation = atan2(-(x_axis[0] - origin[0]), x_axis[1]-origin[1])

        y_axis = positionToMap((0, 1))
        axis_test = atan2(-(y_axis[0] - origin[0]), y_axis[1]-origin[1])
        angle_xy = np.unwrap([0, axis_test - self.axis_orientation])[1]
        assert angle_xy > 0, "positionToMap does not define a right-handed coordinate system"

    def UpdateMapDisplay(self, new_map, position, orientation=None, extra_img=None):
        """
        Updates the internally stored map image using `new_map` and displays
        the updated map along with the initial position of the robot, (0,0),
        and the current position of the robot (`position`). If `orientation`
        is supplied, the initial and current orientation will also be
        displayed.  If `extra_image` is supplied, the image will be displayed
        alongside the map.
        `new_map` must be the same size as the original map (30, 40).
        `extra_image` must be None or have shape (480, 640, 3).
        """
        assert new_map.shape == self.map_size, "New map size doesn't match old map size"
        assert extra_img is None or extra_img.shape == self.drawn_map.shape, "Extra image must be shape (480, 640, 3)"
        for ii in np.ndindex(self.map_size):
            if self.map[ii] == int(new_map[ii]):
                continue
            self.map[ii] = int(new_map[ii])
            pt1 = (int(self.draw_scale*ii[1]), int(self.draw_scale*ii[0]))
            pt2 = (int(self.draw_scale*(ii[1]+ 1)), int(self.draw_scale*(ii[0] + 1)))
            color = self.map_colors[self.map[ii] + 1]
            cv2.rectangle(self.drawn_map, pt1, pt2, color, -1)

        img = np.copy(self.drawn_map)
        arrow_size = 10

        start_pos = self.positionToMap((0,0))
        start_pos = (int(self.draw_scale*start_pos[1]),
                     int(self.draw_scale*start_pos[0]))
        cv2.circle(img, start_pos, self.draw_scale - 2, [0, 1, 0], -1)

        if orientation is not None:
            arrow_start = (int(start_pos[0] - arrow_size*cos(self.axis_orientation)),
                           int(start_pos[1] + arrow_size*sin(self.axis_orientation)))
            arrow_end = (int(start_pos[0] + arrow_size*cos(self.axis_orientation)),
                         int(start_pos[1] - arrow_size*sin(self.axis_orientation)))
            cv2.line(img, arrow_start, arrow_end, [0, 0, 0], 1)

        current_pos = self.positionToMap(position)
        current_pos = (int(self.draw_scale*current_pos[1]),
                       int(self.draw_scale*current_pos[0]))
        cv2.circle(img, current_pos, self.draw_scale - 2, [0, 0, 1], -1)

        if orientation is not None:
            current_oriention = self.axis_orientation + orientation
            arrow_start = (int(current_pos[0] - arrow_size*cos(current_oriention)),
                            int(current_pos[1] + arrow_size*sin(current_oriention)))
            arrow_end = (int(current_pos[0] + arrow_size*cos(current_oriention)),
                          int(current_pos[1] - arrow_size*sin(current_oriention)))
            cv2.line(img, arrow_start, arrow_end, [0, 0, 0], 1)

        if extra_img is not None:
            img = np.hstack((img, extra_img))

        cv2.imshow('Map', img)
        cv2.waitKey(5)

    def SaveMap(self, filename, position, orientation=None):
        """
        Saves the stored map to file `filename`, with the initial position of
        the robot, (0,0), and the current position of the robot (`position`)
        included on the map.  If `orientation` is supplied, the initial and
        current orientation will also be included in the saved image.
        """
        img = np.copy(self.drawn_map)
        arrow_size = 10

        start_pos = self.positionToMap((0,0))
        start_pos = (int(self.draw_scale*start_pos[1]),
                     int(self.draw_scale*start_pos[0]))
        cv2.circle(img, start_pos, self.draw_scale - 2, [0, 1, 0], -1)

        if orientation is not None:
            arrow_start = (int(start_pos[0] - arrow_size*cos(self.axis_orientation)),
                           int(start_pos[1] + arrow_size*sin(self.axis_orientation)))
            arrow_end = (int(start_pos[0] + arrow_size*cos(self.axis_orientation)),
                         int(start_pos[1] - arrow_size*sin(self.axis_orientation)))
            cv2.line(img, arrow_start, arrow_end, [0, 0, 0], 1)

        current_pos = self.positionToMap(position)
        current_pos = (int(self.draw_scale*current_pos[1]),
                       int(self.draw_scale*current_pos[0]))
        cv2.circle(img, current_pos, self.draw_scale - 2, [0, 0, 1], -1)

        if orientation is not None:
            current_oriention = self.axis_orientation + orientation
            arrow_start = (int(current_pos[0] - arrow_size*cos(current_oriention)),
                           int(current_pos[1] + arrow_size*sin(current_oriention)))
            arrow_end = (int(current_pos[0] + arrow_size*cos(current_oriention)),
                         int(current_pos[1] - arrow_size*sin(current_oriention)))
            cv2.line(img, arrow_start, arrow_end, [0, 0, 0], 1)

        cv2.imwrite(filename, img)


if __name__ == '__main__':
    import time

    def DummyPositionToMap(position):
        meters_to_grid = 0.3
        x_steps = position[0] / meters_to_grid
        y_steps = position[1] / meters_to_grid
        return (-x_steps + 25, -y_steps + 20)

    mapper = MapDrawer(DummyPositionToMap)
    my_map = -np.ones((30,40))

    # Sending first image twice as the first image drawn does not fully render
    mapper.UpdateMapDisplay(my_map, (0, 0))
    mapper.UpdateMapDisplay(my_map, (0, 0))
    time.sleep(3)

    start_pos = (0, 0)
    start_pos_map = DummyPositionToMap(start_pos)
    my_map[start_pos_map[0], start_pos_map[1]] = 0
    my_map[start_pos_map[0]-1, start_pos_map[1]] = 0
    my_map[start_pos_map[0]-1, start_pos_map[1]-1] = 0
    my_map[start_pos_map[0], start_pos_map[1]-1] = 0

    mapper.UpdateMapDisplay(my_map, start_pos)
    time.sleep(3)

    end_pos = (1.8, 0.6)
    end_pos_map = DummyPositionToMap(end_pos)
    my_map[end_pos_map[0], end_pos_map[1]] = 0
    my_map[end_pos_map[0]-1, end_pos_map[1]] = 0
    my_map[end_pos_map[0]-1, end_pos_map[1]-1] = 0
    my_map[end_pos_map[0], end_pos_map[1]-1] = 0

    my_map[18:24, 19:21] = 0
    my_map[15:20, 30:35] = 1

    mapper.UpdateMapDisplay(my_map, end_pos, radians(30))
    time.sleep(3)

    mapper.SaveMap("test_map.png", end_pos, radians(30))

    try:
        mapper.UpdateMapDisplay(my_map[:20], end_pos)
    except Exception, err:
        print err

    try:
        mapper.UpdateMapDisplay(my_map, end_pos, extra_img=np.zeros((480, 640)))
    except Exception, err:
        print err
