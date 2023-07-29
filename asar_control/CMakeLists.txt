cmake_minimum_required(VERSION 3.0.2)
project(asar_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -fPIC)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC"  )

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  std_msgs
  geometry_msgs
  eigen_conversions
  dynamic_reconfigure
  joint_limits_interface
  actionlib_msgs
  actionlib
  message_generation
  codcs_ik
  gen3_control
  forceps_control
  effort_controllers
)
## System dependencies are found with CMake's conventions
# Boost
find_package(Boost REQUIRED COMPONENTS date_time)

# Pinnochio
# list(APPEND CMAKE_PREFIX_PATH "/opt/openrobots/")
find_package(pinocchio REQUIRED)

# NLOPT
find_package(PkgConfig REQUIRED)
pkg_check_modules(pkg_nlopt REQUIRED nlopt)

find_package(codcs_ik REQUIRED)

# Reflexxes
set(EXTRALIB_BIN1 ${PROJECT_SOURCE_DIR}/lib/libReflexxesTypeII.a)

# System dependencies are found with CMake's conventions
find_package(Qt5Widgets REQUIRED)

set (QT_PROJECT_SOURCE_DIR src/gui)

set (QtInterfaceLib_src ${QT_PROJECT_SOURCE_DIR}/mainwindow.cpp)
set (QtInterfaceLib_hdr ${QT_PROJECT_SOURCE_DIR}/mainwindow.h)
set (QtInterfaceLib_ui  ${QT_PROJECT_SOURCE_DIR}/mainwindow.ui)
set (QtInterfaceLib_qrc ${QT_PROJECT_SOURCE_DIR}/resource_file.qrc)

qt5_wrap_cpp(QtInterfaceLib_hdr_moc ${QtInterfaceLib_hdr})
qt5_wrap_ui (QtInterfaceLib_ui_moc  ${QtInterfaceLib_ui})
qt5_add_resources(QtInterfaceLib_qrc_moc ${QtInterfaceLib_qrc})

include_directories (${QT_PROJECT_SOURCE_DIR})
include_directories (${PROJECT_BINARY_DIR})

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
  # FILES
  # PhantomButtonEvent.msg
#   in_virtuose_force.msg
#   in_virtuose_pose.msg
#   in_virtuose_speed.msg
#   out_virtuose_force.msg
#   out_virtuose_physical_pose.msg
#   out_virtuose_pose.msg
#   out_virtuose_speed.msg
#   out_virtuose_status.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   robot_request.srv
# )

## Generate actions in the 'action' folder
add_action_files(
  FILES
  FollowCartesianTarget.action
  FollowJointTarget.action
  SolveIK.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  sensor_msgs
  geometry_msgs
  actionlib_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

generate_dynamic_reconfigure_options(
  config/DynVar.cfg
)

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
 INCLUDE_DIRS include include/Reflexxes
 LIBRARIES asar_control_lib 
 CATKIN_DEPENDS std_msgs geometry_msgs message_runtime actionlib actionlib_msgs roscpp 
#  DEPENDS codcs_ik
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  include/Reflexxes
  ${catkin_INCLUDE_DIRS}
  ${pinocchio_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  # ${pkg_nlopt_INCLUDE_DIRS}

)

# Declare a C++ library
add_library(${PROJECT_NAME}_lib
  src/asar_control.cpp
)

add_library(asar_planner_lib
  src/asar_planner.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME}_lib ${${PROJECT_NAME}_generate_messages} ${${PROJECT_NAME}_gencfg} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/main.cpp)

add_executable(asar_gui_node
  src/asar_gui.cpp
  ${QT_PROJECT_SOURCE_DIR}/mainwindow.cpp
  ${QtInterfaceLib_src}
  ${QtInterfaceLib_hdr_moc}
  ${QtInterfaceLib_ui_moc}
  ${QtInterfaceLib_qrc_moc}
  )

add_executable(asar_control_main
  src/asar_control_main.cpp
)


# add_executable(asar_teleop_virtuose_node
#   src/asar_teleop_virtuose.cpp
# )

# add_executable(asar_teleop_omega_node
#   src/asar_teleop_omega.cpp
# )

# add_executable(asar_clrmpc_node
#   src/asar_clrmpc.cpp
# )

# add_executable(asar_triangulation_node
#   src/asar_triangulation.cpp
# )

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
  pinocchio::pinocchio 
  ${codcs_ik_LIBRARIES}
)

target_link_libraries(asar_planner_lib
  ${catkin_LIBRARIES}
  pinocchio::pinocchio 
  ${codcs_ik_LIBRARIES}
)

target_link_libraries(asar_gui_node
 ${catkin_LIBRARIES}
 ${CMAKE_THREAD_LIBS_INIT} # needed for the qrc file compilation
 Qt5::Widgets
)

target_link_libraries(asar_control_main
  ${catkin_LIBRARIES}
  pinocchio::pinocchio 
  ${PROJECT_NAME}_lib
  ${codcs_ik_LIBRARIES}
  casadi
  ${pkg_nlopt_LIBRARIES}
  ${EXTRALIB_BIN1}
)

# target_link_libraries(asar_teleop_virtuose_node
#   ${catkin_LIBRARIES}
#   pinocchio::pinocchio 
#   ${PROJECT_NAME}_lib
#   ${codcs_ik_LIBRARIES}
#   casadi
#   Eigen3::Eigen
#   ${pkg_nlopt_LIBRARIES}
#   ${EXTRALIB_BIN1}
# )

# target_link_libraries(asar_teleop_omega_node
#   ${catkin_LIBRARIES}
#   pinocchio::pinocchio 
#   ${PROJECT_NAME}_lib
#   ${codcs_ik_LIBRARIES}
#   casadi
#   Eigen3::Eigen
#   ${pkg_nlopt_LIBRARIES}
#   ${EXTRALIB_BIN1}
# )

# target_link_libraries(asar_clrmpc_node
#   ${catkin_LIBRARIES}
#   pinocchio::pinocchio 
#   ${PROJECT_NAME}_lib
#   ${codcs_ik_LIBRARIES}
#   casadi
#   Eigen3::Eigen
#   ${pkg_nlopt_LIBRARIES}
#   ${EXTRALIB_BIN1}
# )

# target_link_libraries(asar_triangulation_node
#   ${catkin_LIBRARIES}
#   pinocchio::pinocchio 
#   ${PROJECT_NAME}_lib
#   ${codcs_ik_LIBRARIES}
#   casadi
#   Eigen3::Eigen
#   ${pkg_nlopt_LIBRARIES}
#   ${EXTRALIB_BIN1}
# )





#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_kinova_asar_control.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
