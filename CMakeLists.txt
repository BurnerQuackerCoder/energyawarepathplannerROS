cmake_minimum_required(VERSION 3.0.2)
project(enawpl)

find_package(catkin REQUIRED
  COMPONENTS
    cmake_modules
    costmap_2d
    geometry_msgs
    message_generation
    nav_core
    nav_msgs
    pluginlib
    rosconsole
    roscpp
    tf2_ros
    sensor_msgs
    visualization_msgs
)

find_package(Eigen3 REQUIRED)

add_service_files(
  DIRECTORY srv
  FILES
    MakeNavPlan.srv
    SetCostmap.srv
)

generate_messages(
  DEPENDENCIES
    geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES enawpl
  CATKIN_DEPENDS
    geometry_msgs
    message_runtime
    nav_core
    nav_msgs
    pluginlib
    roscpp
    sensor_msgs
    visualization_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_definitions(${EIGEN3_DEFINITIONS})

add_library(enawpl
  src/navfn.cpp
  src/navfn_ros.cpp
)

target_link_libraries(enawpl
  ${catkin_LIBRARIES}
)

add_dependencies(enawpl
  ${PROJECT_NAME}_generate_messages_cpp
  ${catkin_EXPORTED_TARGETS}
)

add_executable(navfn_node src/navfn_node.cpp)

target_link_libraries(navfn_node
  enawpl
  ${catkin_LIBRARIES}
)

install(TARGETS enawpl navfn_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES bgp_plugin_jp.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
