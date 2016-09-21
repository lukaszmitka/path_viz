#cmake_minimum_required(VERSION 2.8.3)
cmake_minimum_required(VERSION 3.5.1)
project(path_viz)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++0x")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        nav_msgs
        rosbag
        roscpp
        tf
        )


catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES path_viz
        #  CATKIN_DEPENDS nav_msgs rosbag roscpp
        #  DEPENDS system_lib
)
include_directories(
        ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(path_viz src/main.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(path_viz_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(path_viz
        ${catkin_LIBRARIES}
        )
