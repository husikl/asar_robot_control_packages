cmake_minimum_required(VERSION 2.8.3)
project(mc_daq_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH} /opt/local/lib)

#set(EXTRALIB_BIN ${PROJECT_SOURCE_DIR}/lib/libbiodaq.so)

# add_library(uldaq SHARED)
find_library(ULDAQ_LIB uldaq)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


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
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
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
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   daq_cmd.srv
  )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES usb1608 usb3104 usbquad08
  CATKIN_DEPENDS message_runtime roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(pci1716
#   src/${PROJECT_NAME}/pci1716.cpp
# )
# add_library(usb1608 src/usb1608/usb1608.cpp)
# add_library(usb3104 src/usb3104/usb3104.cpp)
# add_library(usbquad08 src/usbquad08/usbquad08.cpp)

# ## Add cmake target dependencies of the library
# ## as an example, code may need to be generated before libraries
# ## either from message generation or dynamic reconfigure
# # add_dependencies(pci1716 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# ## Declare a C++ executable

# add_executable(mc_daqlib_sample src/daqlib_sample.cpp)
# add_executable(mc_daq_forceps src/forceps_demo.cpp)
# add_executable(mc_daq_haptic src/haptic_main.cpp)
# add_executable(mc_daq_usb1608_stream src/usb1608_stream.cpp )
# add_executable(mc_daq_usb3104_stream src/usb3104_stream.cpp )
# add_executable(mc_daq_usbquad08_stream src/usbquad08_stream.cpp )


# ## Add cmake target dependencies of the executable
# ## same as for the library above
# add_dependencies(mc_daqlib_sample ${${PROJECT_NAME}_generate_messages_cpp} ${catkin_EXPORTED_TARGETS})

# ## Specify libraries to link a library or executable target against
# target_link_libraries(usb1608
#   ${ULDAQ_LIB}
#   ${catkin_LIBRARIES}
#   uldaq
#   #   ${EXTRALIB_BIN}
# )

# target_link_libraries(usb3104
#   ${ULDAQ_LIB}
#   ${catkin_LIBRARIES}
#   uldaq
#   #   ${EXTRALIB_BIN}
# )

# target_link_libraries(usbquad08
#   ${ULDAQ_LIB}
#   ${catkin_LIBRARIES}
#   uldaq
#   #   ${EXTRALIB_BIN}
# )


# target_link_libraries(mc_daqlib_sample
#   usb1608
#   usb3104
#   usbquad08
#   ${ULDAQ_LIB}
#   ${catkin_LIBRARIES}
#   #   ${EXTRALIB_BIN}
# )

# target_link_libraries(mc_daq_forceps
#   usb1608
#   usb3104
#   usbquad08
#   ${ULDAQ_LIB}
#   ${catkin_LIBRARIES}
#   #   ${EXTRALIB_BIN}
# )

# target_link_libraries(mc_daq_haptic
#   usb1608
#   usb3104
#   ${ULDAQ_LIB}
#   ${catkin_LIBRARIES}
#   #   ${EXTRALIB_BIN}
# )

# target_link_libraries(mc_daq_usb1608_stream
#   usb1608
#   ${ULDAQ_LIB}
#   ${catkin_LIBRARIES}
# )

# target_link_libraries(mc_daq_usb3104_stream
#   usb3104
#   ${ULDAQ_LIB}
#   ${catkin_LIBRARIES}
# )

# target_link_libraries(mc_daq_usbquad08_stream
#   usbquad08
#   ${ULDAQ_LIB}
#   ${catkin_LIBRARIES}
# )
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS pci1716 pci1716_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_pci1716.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
