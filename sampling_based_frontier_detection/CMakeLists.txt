cmake_minimum_required(VERSION 3.1)
project(sampling_based_frontier_detection)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  costmap_2d
  geometry_msgs
  map_msgs
  move_base_msgs
  nav_msgs
  roscpp
  std_msgs
  tf
  visualization_msgs
)
if(NOT Boost_FOUND)
    message("Not found Boost")
endif()

include_directories(${Boost_INCLUDE_DIRS}
include)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    actionlib
    actionlib_msgs
    costmap_2d
    geometry_msgs
    map_msgs
    move_base_msgs
    nav_msgs
    roscpp
    std_msgs
    tf
    visualization_msgs
)
find_package(Boost)
if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
endif()

###########
## Build ##
###########
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)


## Declare a C++ executable
add_executable(sampling_based_frontier_detection src/sampling_based_frontier_detection.cpp)
add_executable(test_map_pub src/test_map_pub.cpp)


## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(robot_exploration_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against

 target_link_libraries(sampling_based_frontier_detection
 ${catkin_LIBRARIES}
)
target_link_libraries(test_map_pub
${catkin_LIBRARIES}
)
#############
## Install ##
#############



# install roslaunch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)

  # test all launch files
  roslaunch_add_file_check(launch)
endif()
