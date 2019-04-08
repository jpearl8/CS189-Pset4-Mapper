# pset-4
The Autonomous Mapper


Our solution is in b1_wander_test.py.

In the class-initializer function, globals are initialized and modules are subscribed to data from EKFs, twist objects, depth processing, and bumper processing. 

To initialize and populate the map, there are functions following the class-initializer function that convert the robot's EKF position to coordinates, initialize the map, and update positions on the map to be free or occupied. We have reversed the orientation of the map array and map array than from the (30,40) that was specified in the original map_util.py function. The update free and update occupied functions both are dependent upon globals. The free function takes the global of the robot's position and marks that in the map as free. The occupied function takes the global of the robot's detected object and that marks that in the map as occupied. 

For the wander function, we pieced together the process_depth and bumper sensor functions from previous psets. While the robot is still running, if there is no obstacle detected by the depth camera or no initialized a bump event, the current position will be updated on the map to represent a free location (via the updateMapFree function). If there is an obstacle or the bump sensor initializes a bump event, the position of the obstacle will be added to the map (via the updateMapOccupied function). The difference between finding an obstacle via depth sensors verses via bump sensors is present in calculating the distance from the robot that is marked as occupied (the same mathematic calculations are used, just objects found via bump sensors will be assumed to be closer). To move away from the obstacles, if the obstacle was detected by the depth camera, the robot will turn in the opposite direction of the object. It is skewed to favor the right side to avoid getting stuck with an object directly in front. If the obstacle was detected by a bump event, then the robot will back up, and turn in the opposite direction to the side of the bump to move away from the obstacle. Below the wander function are the variuos process events functions from past psets like process_bump_sensing, process_depth_image, bound_object, as well as the new process_ekf function. In the shutdown function, the robot saves the current map as a png file.
  


