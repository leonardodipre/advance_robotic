cmake_minimum_required(VERSION 2.8.3)
cmake_policy(SET CMP0048 NEW)
project(arm_controllers)

## Compile as C++11, supported in ROS Noetic
add_compile_options(-std=c++11)

# Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  angles
  controller_interface
  control_msgs
  urdf
  kdl_parser
  realtime_tools
  dynamic_reconfigure
  roscpp
  std_msgs
  rospy
  message_generation
  geometry_msgs
  visualization_msgs
)

# Dynamic reconfigure
generate_dynamic_reconfigure_options(
  cfg/PassivityControllerParams.cfg
)

# Message files
add_message_files(
  FILES
    ControllerJointState.msg
    ArucoTracker.msg
    ExtendedVector3.msg  # Define custom message here
    TrajectoryPhase.msg  # New message
    TrajectoryPlan.msg   # New message
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

# Declare catkin package
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    angles
    controller_interface
    control_msgs
    urdf
    kdl_parser
    realtime_tools
    dynamic_reconfigure
    roscpp
    std_msgs
    message_runtime
    geometry_msgs
    visualization_msgs
)

# Include directories
include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

# Declare the library
add_library(${PROJECT_NAME}
  src/time_delay_controller.cpp
  src/gravity_comp_controller.cpp
  src/computed_torque_controller.cpp
  src/computed_torque_clik_controller.cpp
  src/adaptive_variable_impedance_controller.cpp
  src/passivity_controller.cpp
)

# Link the library with catkin and system dependencies
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# Add dependencies for message generation and dynamic reconfigure
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp  # Dipendenza per la generazione dei messaggi
)

# Declare the executable
add_executable(motion_switcher_node src/motion_switcher_node.cpp)

# Link the executable with catkin dependencies
target_link_libraries(motion_switcher_node
  ${catkin_LIBRARIES}
)

# Add dependencies for the executable
add_dependencies(motion_switcher_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  ${PROJECT_NAME}_generate_messages_cpp  # Dipendenza per la generazione dei messaggi
)

# Install C++ executables and libraries
install(TARGETS ${PROJECT_NAME} motion_switcher_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

# Install Python scripts
catkin_install_python(PROGRAMS 
  scripts/movment_publisher.py  # Correct the name if necessary
  scripts/controll_publisher.py
  scripts/potential_forces.py
  scripts/arunco_vs.py
  scripts/aruco_movment.py
  scripts/trajectory_planning.py
  scripts/depth.py

  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
