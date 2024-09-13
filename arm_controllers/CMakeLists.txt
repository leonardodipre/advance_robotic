cmake_policy(SET CMP0048 NEW)
cmake_minimum_required(VERSION 2.8.3)
project(arm_controllers)

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
  rospy  # Added for Python script support
)

# Dynamic reconfigure
generate_dynamic_reconfigure_options(
  cfg/PassivityControllerParams.cfg
)

# Message generation
add_message_files(FILES ControllerJointState.msg)
generate_messages(DEPENDENCIES std_msgs)

# Include directories
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS})

# Declare the catkin package
catkin_package(
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

# Link the library with catkin dependencies
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# Declare the executable
add_executable(motion_switcher_node src/motion_switcher_node.cpp)

# Link the executable with catkin dependencies
target_link_libraries(motion_switcher_node ${catkin_LIBRARIES})

# Add dependencies for dynamic reconfigure and message generation
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_gencfg)
add_dependencies(motion_switcher_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# Installation for C++ files
install(TARGETS ${PROJECT_NAME} motion_switcher_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


catkin_install_python(PROGRAMS 
  scripts/joint_state_printer.py
  scripts/movment_publisher.py  # Added your new Python file
  scripts/subs.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
