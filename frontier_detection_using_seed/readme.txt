after playing the bag containing the costmap and tf
( 
cd ~/0project/exploration_project/bag
rosbag play 2021-08-01-07-28-40.bag  --clock --pause
)


command:roslaunch frontier_detection frontier_detection_new_test_version.launch   using the naivest method to get the frontier(global map with 8s and local map with 6ms,searching every grid in the map)


