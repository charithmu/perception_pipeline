cmake_minimum_required(VERSION 2.8.3)
project(perception_pipeline)
add_definitions(-std=c++14)

# Find catkin macros and libraries if COMPONENTS list like find_package(catkin
# REQUIRED COMPONENTS xyz) is used, also find other catkin packages
find_package(
  catkin REQUIRED
  COMPONENTS roscpp
             sensor_msgs
             tf
             tf2
             pcl_msgs
             pcl_ros
             pcl_conversions
             visualization_msgs
             tf_conversions
             tf2_geometry_msgs
             std_msgs
             message_generation)
find_package(PCL REQUIRED)

# System dependencies are found with CMake's conventions find_package(Boost
# REQUIRED COMPONENTS system)

# Uncomment this if the package has a setup.py. This macro ensures modules and
# global scripts declared therein get installed See
# http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

# ##############################################################################
# Declare ROS messages, services and actions ##
# ##############################################################################

# To declare and build messages, services or actions from within this package,
# follow these steps: * Let MSG_DEP_SET be the set of packages whose message
# types you use in your messages/services/actions (e.g. std_msgs,
# actionlib_msgs, ...). * In the file package.xml: * add a build_depend and a
# run_depend tag for each package in MSG_DEP_SET * If MSG_DEP_SET isn't empty
# the following dependencies might have been pulled in transitively but can be
# declared for certainty nonetheless: * add a build_depend tag for
# "message_generation" * add a run_depend tag for "message_runtime" * In this
# file (CMakeLists.txt): * add "message_generation" and every package in
# MSG_DEP_SET to find_package(catkin REQUIRED COMPONENTS ...) * add
# "message_runtime" and every package in MSG_DEP_SET to
# catkin_package(CATKIN_DEPENDS ...) * uncomment the add_*_files sections below
# as needed and list every .msg/.srv/.action file to be processed * uncomment
# the generate_messages entry below * add every package in MSG_DEP_SET to
# generate_messages(DEPENDENCIES ...)

# Generate messages in the 'msg' folder
add_message_files(FILES Quadcloud.msg)

# Generate services in the 'srv' folder
add_service_files(FILES PredictLabels.srv)

# Generate actions in the 'action' folder add_action_files( FILES Action1.action
# Action2.action )

# Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES std_msgs sensor_msgs)

# ##############################################################################
# catkin specific configuration ##
# ##############################################################################
# The catkin_package macro generates cmake config files for your package Declare
# things to be passed to dependent projects INCLUDE_DIRS: uncomment this if you
# package contains header files LIBRARIES: libraries you create in this project
# that dependent projects also need CATKIN_DEPENDS: catkin_packages dependent
# projects also need DEPENDS: system dependencies of this project that dependent
# projects also need
catkin_package(
  # INCLUDE_DIRS include LIBRARIES lesson_perception CATKIN_DEPENDS roscpp
  # sensor_msgs tf DEPENDS system_lib
  CATKIN_DEPENDS message_runtime)

# ##############################################################################
# Build ##
# ##############################################################################

# Specify additional locations of header files Your package locations should be
# listed before other locations include_directories(include)
include_directories(src ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

# Declare a cpp library add_library(lesson_perception
# src/${PROJECT_NAME}/lesson_perception.cpp )

# Declare a cpp executable
add_executable(perception_node src/perception_node.cpp)
add_executable(semseg_node src/semseg_node.cpp)
add_executable(static_transforms src/static_transforms.cpp)
add_executable(sensor_playback src/sensor_playback.cpp)
add_executable(sync_node src/sync_node.cpp)

# Add cmake target dependencies of the executable/library as an example, message
# headers may need to be generated before nodes
# add_dependencies(lesson_perception_node
# lesson_perception_generate_messages_cpp)

# Specify libraries to link a library or executable target against
target_link_libraries(perception_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(semseg_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(static_transforms ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(sensor_playback ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(sync_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})

# ##############################################################################
# Install ##
# ##############################################################################

catkin_install_python(
  PROGRAMS
  scripts/talker.py
  scripts/listener.py
  scripts/pointcloud_publisher.py
  scripts/prediction_service.py
  DESTINATION
  ${CATKIN_PACKAGE_BIN_DESTINATION})

# all install targets should use catkin DESTINATION variables See
# http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

# Mark executable scripts (Python etc.) for installation in contrast to
# setup.py, you can choose the destination install(PROGRAMS
# scripts/my_python_script DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )

# Mark executables and/or libraries for installation install(TARGETS
# lesson_perception lesson_perception_node ARCHIVE DESTINATION
# ${CATKIN_PACKAGE_LIB_DESTINATION} LIBRARY DESTINATION
# ${CATKIN_PACKAGE_LIB_DESTINATION} RUNTIME DESTINATION
# ${CATKIN_PACKAGE_BIN_DESTINATION} )

# Mark cpp header files for installation install(DIRECTORY
# include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# FILES_MATCHING PATTERN "*.h" PATTERN ".svn" EXCLUDE )

# Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES # myfile1 # myfile2 DESTINATION
# ${CATKIN_PACKAGE_SHARE_DESTINATION} )

# ##############################################################################
# Testing ##
# ##############################################################################

# Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_lesson_perception.cpp)
# if(TARGET ${PROJECT_NAME}-test) target_link_libraries(${PROJECT_NAME}-test
# ${PROJECT_NAME}) endif()

# Add folders to be run by python nosetests catkin_add_nosetests(test)

# message("CMAKE_C_FLAGS_DEBUG is ${CMAKE_C_FLAGS_DEBUG}")
# message("CMAKE_C_FLAGS_RELEASE is ${CMAKE_C_FLAGS_RELEASE}")
# message("CMAKE_C_FLAGS_RELWITHDEBINFO is ${CMAKE_C_FLAGS_RELWITHDEBINFO}")
# message("CMAKE_C_FLAGS_MINSIZEREL is ${CMAKE_C_FLAGS_MINSIZEREL}")

# message("CMAKE_CXX_FLAGS_DEBUG is ${CMAKE_CXX_FLAGS_DEBUG}")
# message("CMAKE_CXX_FLAGS_RELEASE is ${CMAKE_CXX_FLAGS_RELEASE}")
# message("CMAKE_CXX_FLAGS_RELWITHDEBINFO is ${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")
# message("CMAKE_CXX_FLAGS_MINSIZEREL is ${CMAKE_CXX_FLAGS_MINSIZEREL}")
