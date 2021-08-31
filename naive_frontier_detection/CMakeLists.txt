cmake_minimum_required(VERSION 3.1)
project(naive_frontier_detection)

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



add_library(frontier_detection_lib
src/robot_exploration.cpp
src/frontier_search.cpp
src/pose_handler.cpp
)


add_library(robot_exploration_lib
src/send_goal.cpp
)
target_link_libraries(frontier_detection_lib
  ${catkin_LIBRARIES}
)

target_link_libraries(robot_exploration_lib
  ${catkin_LIBRARIES}
)

## Declare a C++ executable
add_executable(robot_exploration src/send_goal.cpp)
add_executable(frontier_detection1 src/robot_exploration.cpp)


## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(robot_exploration_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against

target_link_libraries(frontier_detection1
  ${catkin_LIBRARIES}
  frontier_detection_lib
)
 target_link_libraries(robot_exploration
   ${catkin_LIBRARIES}
   robot_exploration_lib
 )

#############
## Install ##
#############

# install nodes
install(TARGETS frontier_detection1
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

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
