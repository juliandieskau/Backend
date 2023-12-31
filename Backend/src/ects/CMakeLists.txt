cmake_minimum_required(VERSION 3.0.2)
include(FetchContent)
project(ects VERSION 0.0.1 DESCRIPTION "Extensible Control and Telemetry System")

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++2a -Wall -Wextra -Wno-unused-parameter)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  spot_msgs
  iosb_localization_filter
  iosb_nav_msgs
  message_generation
)



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
add_message_files(
   FILES
   # core
   ForceRetransmit.msg
   # waypoints
   AddWaypoint.msg
   RemoveWaypoint.msg
   ReorderWaypoints.msg
   ReplaceWaypoint.msg
   Waypoint.msg
   WaypointList.msg

   # systemmonitor
   Aggregation.msg
   CpuUsage.msg
   CpuUsageHistory.msg
   DiskUsage.msg
   DiskUsageHistory.msg
   MemoryUsage.msg
   MemoryUsageHistory.msg
   NetworkInfo.msg
   NetworkUsage.msg
   NetworkUsageHistory.msg
   ProcessTotal.msg
   ProcessTotalHistory.msg
)

## Generate services in the 'srv' folder
add_service_files(
    FILES
    # core
    ECTSStatus.srv
    # waypoints
    WaypointListDirectory.srv
    WaypointListFile.srv
    # systemmonitor
    AdapterList.srv
    AggregationList.srv
    MountpointList.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

# Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   sensor_msgs
   spot_msgs
   iosb_localization_filter
   iosb_nav_msgs
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

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

set(ECTS_PLUGINS
        ects_plugin_core
        ects_plugin_battery
        ects_plugin_control
        ects_plugin_waypoints
        ects_plugin_systemmonitor
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
  INCLUDE_DIRS include
  LIBRARIES ${ECTS_PLUGINS}
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/ects.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/ects_node.cpp)

add_library(ects_plugin_core SHARED 
        plugins/core/PluginCore.cpp
        plugins/core/CoreMessages.cpp
)

add_library(ects_plugin_battery SHARED 
        plugins/battery/BatteryMonitor.cpp
        plugins/battery/BatteryMessages.cpp
)

add_library(ects_plugin_control SHARED
        plugins/control/Control.cpp
)

add_library(ects_plugin_position SHARED
        plugins/position/Position.cpp
)

add_library(ects_plugin_systemmonitor SHARED
        plugins/systemmonitor/SystemMonitor.cpp

        plugins/systemmonitor/aggregations/Aggregation.cpp
        plugins/systemmonitor/cpu/Cpu.cpp
        plugins/systemmonitor/disk/Disk.cpp
        plugins/systemmonitor/memory/Memory.cpp
        plugins/systemmonitor/network/Network.cpp
        plugins/systemmonitor/programs/Programs.cpp

        plugins/systemmonitor/aggregations/AggregationMessage.cpp
        plugins/systemmonitor/cpu/CpuMessages.cpp
        plugins/systemmonitor/disk/DiskMessages.cpp
        plugins/systemmonitor/memory/MemoryMessages.cpp
        plugins/systemmonitor/network/NetworkMessages.cpp
        plugins/systemmonitor/programs/ProgramsMessages.cpp
)

add_library(ects_plugin_waypoints SHARED
        plugins/waypoint/WaypointManager.cpp
        plugins/waypoint/Waypoint.cpp
        plugins/waypoint/WaypointList.cpp
        plugins/waypoint/WaypointMessages.cpp
        plugins/waypoint/WaypointListStorage.cpp
        plugins/waypoint/WaypointListFileMessages.cpp
)

set_target_properties(ects_plugin_core PROPERTIES PUBLIC_HEADER "include/Plugin.hpp")
add_dependencies(ects_plugin_core ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
set_target_properties(ects_plugin_battery PROPERTIES PUBLIC_HEADER "include/Plugin.hpp")
add_dependencies(ects_plugin_battery ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
set_target_properties(ects_plugin_control PROPERTIES PUBLIC_HEADER "include/Plugin.hpp")
add_dependencies(ects_plugin_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
set_target_properties(ects_plugin_position PROPERTIES PUBLIC_HEADER "include/Plugin.hpp")
add_dependencies(ects_plugin_position ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
set_target_properties(ects_plugin_systemmonitor PROPERTIES PUBLIC_HEADER "include/Plugin.hpp")
add_dependencies(ects_plugin_systemmonitor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
set_target_properties(ects_plugin_waypoints PROPERTIES PUBLIC_HEADER "include/Plugin.hpp")
add_dependencies(ects_plugin_waypoints ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

set(ECTS_SOURCES 
        src/Configuration.cpp
        src/PluginLoader.cpp
        src/Common.cpp
)

add_executable(${PROJECT_NAME} src/main.cpp ${ECTS_SOURCES})

target_link_libraries(ects ${catkin_LIBRARIES} dl)
## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
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
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

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
catkin_add_gtest(${PROJECT_NAME}-test
        test/RosInterfaceTest.cpp
        test/EctsTest.cpp
        test/WaypointListTest.cpp
        test/WaypointManagerTest.cpp
        test/SystemMonitorTest.cpp
        test/CoreTest.cpp
        test/BatteryTest.cpp
        test/ControlTest.cpp
        test/PositionTest.cpp
        ${ECTS_SOURCES})
add_dependencies(${PROJECT_NAME}-test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES} dl)
  target_link_libraries(${PROJECT_NAME}-test ${ECTS_PLUGINS} dl)
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
