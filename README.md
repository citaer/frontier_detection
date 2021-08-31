# frontier_detection

find the cloeset frontier from robot's current position using global_costmap.

#### command:

#### 1.2D frontier detection in yuqu experiment bag:(frontier search seed is in the last frontier if frontier change quickly, or it is the robot position and bfs searching the closest frontier)

roslaunch frontier_detection frontier_detection_using_seed.launch

#### 3. naive 2D frontier detection with searching all the grid in the map

roslaunch naive_frontier_detection frontier_detection_using_naive_method.launch

#### 2.2D sampling based frontier detection: 

roslaunch sampling_based_frontier_detection frontier_detection_using_sampling_based.launch