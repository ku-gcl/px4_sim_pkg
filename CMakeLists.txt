cmake_minimum_required(VERSION 3.0.2)
project(px4_sim_pkg)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
  rospy
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES px4_sim_pkg
#  CATKIN_DEPENDS geometry_msgs mavros_msgs roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

# add_executable(offboard_sample src/offboard_sample.cpp)
# target_link_libraries(offboard_sample ${catkin_LIBRARIES})

# C++スクリプトをインストールするための設定
add_executable(ardu_guided src/ardu_guided.cpp)
target_link_libraries(ardu_guided ${catkin_LIBRARIES})

add_executable(pose_transformer src/pose_transformer.cpp)
target_link_libraries(pose_transformer ${catkin_LIBRARIES})

# Pythonスクリプトをインストールするための設定
catkin_install_python(PROGRAMS
  scripts/dmd_test_circlexy.py
  scripts/dmd_test_eightxz.py
  scripts/dmd_test_rightleft.py
  scripts/dmd_test_updown.py
  scripts/dmd_train_circle.py
  scripts/pose_transformer.py
  scripts/guided_hover_test.py
  scripts/guided_circle_test.py
  scripts/rtdmd_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
