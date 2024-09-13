cmake_minimum_required(VERSION 3.0.2)
project(eigen_kdl_examples)


## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp kdl_parser)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(kdl src/kdl.cpp)
target_link_libraries(kdl ${catkin_LIBRARIES})
#add_dependencies(kdl kdl_parser)

add_executable(eigen src/eigen.cpp)
target_link_libraries(eigen ${catkin_LIBRARIES})
