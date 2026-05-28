cmake_minimum_required(VERSION 3.0.2)
project(ins_gt_logger_ros C CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  std_msgs
  tf2
  tf2_ros
)
find_package(GeographicLib REQUIRED)
find_package(Threads REQUIRED)

set(SDK_DIR "${CMAKE_CURRENT_SOURCE_DIR}/third_party/inertiallabs_sdk")

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GeographicLib_INCLUDE_DIRS}
  ${SDK_DIR}/include
)

add_library(ins_gt_inertiallabs_sdk STATIC
  ${SDK_DIR}/src/InertialLabs_INS.c
  ${SDK_DIR}/src/arch/linux/InertialLabs_services.c
)
target_include_directories(ins_gt_inertiallabs_sdk PUBLIC ${SDK_DIR}/include)
target_link_libraries(ins_gt_inertiallabs_sdk PUBLIC Threads::Threads rt)

add_executable(ins_gt_rviz_node src/ins_gt_rviz_node.cpp)
target_link_libraries(ins_gt_rviz_node
  ins_gt_inertiallabs_sdk
  ${catkin_LIBRARIES}
  ${GeographicLib_LIBRARIES}
)

add_executable(ins_gt_convert src/ins_gt_convert.cpp)
target_include_directories(ins_gt_convert PRIVATE ${GeographicLib_INCLUDE_DIRS})
target_link_libraries(ins_gt_convert
  ${GeographicLib_LIBRARIES}
)

install(TARGETS ins_gt_rviz_node ins_gt_convert
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
