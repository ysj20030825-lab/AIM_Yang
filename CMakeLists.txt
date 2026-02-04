cmake_minimum_required(VERSION 3.0.2)
project(aim_hw)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  tf2
  morai_msgs
)
list(APPEND CMAKE_MODULE_PATH "/usr/share/cmake/geographiclib")
find_package(GeographicLib REQUIRED)

add_message_files(
  FILES
  MyData.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime roscpp std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
)

#############
##  Build  ##
#############

add_executable(int32_pub src/1week/int32_pub.cpp)
target_link_libraries(int32_pub ${catkin_LIBRARIES})

add_executable(int32_sub src/1week/int32_sub.cpp)
target_link_libraries(int32_sub ${catkin_LIBRARIES})

add_executable(int32_relay src/1week/int32_relay.cpp)
target_link_libraries(int32_relay ${catkin_LIBRARIES})

add_executable(int32_sub2 src/1week/int32_sub2.cpp)
target_link_libraries(int32_sub2 ${catkin_LIBRARIES})

add_executable(custom_pub src/1week/custom_pub.cpp)
add_dependencies(custom_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(custom_pub ${catkin_LIBRARIES})

add_executable(custom_sub src/1week/custom_sub.cpp)
add_dependencies(custom_sub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(custom_sub ${catkin_LIBRARIES})

add_library(geo_utils src/coordinate/geo_utils.cpp)
target_link_libraries(geo_utils
  ${catkin_LIBRARIES}
  ${GeographicLib_LIBRARIES}
)

add_executable(wgs84_to_utm src/coordinate/wgs84_to_utm.cpp)
target_link_libraries(wgs84_to_utm geo_utils ${catkin_LIBRARIES})

add_executable(wgs84_to_enu src/coordinate/wgs84_to_enu.cpp)
target_link_libraries(wgs84_to_enu geo_utils ${catkin_LIBRARIES})

add_executable(utm_to_enu src/coordinate/utm_to_enu.cpp)
target_link_libraries(utm_to_enu geo_utils ${catkin_LIBRARIES})

add_executable(enu_to_ned src/coordinate/enu_to_ned.cpp)
target_link_libraries(enu_to_ned geo_utils ${catkin_LIBRARIES})

add_executable(pure_pursuit src/path_control/pure_pursuit.cpp)
target_link_libraries(pure_pursuit geo_utils ${catkin_LIBRARIES})

add_executable(stanley src/path_control/stanley.cpp)
target_link_libraries(stanley geo_utils ${catkin_LIBRARIES})