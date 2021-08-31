cmake_minimum_required(VERSION 3.1)
project(frontier_detection_using_seed)

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

# add_executable(frontier_detection
#   src/frontier_detection.cpp
# )
# add_dependencies(frontier_detection ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# target_link_libraries(frontier_detection ${catkin_LIBRARIES} ${Boost_LIBRARIES})


add_executable(frontier_detection_using_seed
  src/frontier_detection_using_seed.cpp
)
add_dependencies(frontier_detection_using_seed ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(frontier_detection_using_seed ${catkin_LIBRARIES} ${Boost_LIBRARIES})

## Declare a C++ executable


## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(robot_exploration_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against

#############
## Install ##
#############

# install nodes


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
