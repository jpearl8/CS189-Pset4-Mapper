# pset-4
The Autonomous Mapper

Active files:
  Initialize and update map - setup.py
  Wander - b1_wander.py
  
Strategy:
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

