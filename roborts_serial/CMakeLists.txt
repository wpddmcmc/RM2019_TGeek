project(roborts_serial)
cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roborts_msgs
  )

catkin_package()

include_directories(
  "sdk"
  ${catkin_INCLUDE_DIRS}
)

add_executable(serial_node
  serial/serial_node.cpp
  sdk/serial_device.cpp
)

target_link_libraries(serial_node PUBLIC
  ${catkin_LIBRARIES})

target_include_directories(serial_node PUBLIC ${catkin_INCLUDE_DIRS})

add_dependencies(serial_node roborts_msgs_generate_messages)
