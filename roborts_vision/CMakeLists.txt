project(roborts_vision)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_module)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  roborts_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS
)

include_directories(
  "camera"
  "detect_factory"
  "executor"
  ${OpenCV_INCLUDE_DIRECTORIES}
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(camera)
add_subdirectory(detect_factory)
add_subdirectory(executor)

add_executable(${PROJECT_NAME}_node
  GXnode/vision_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  PRIVATE
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  camera_LIBS
  detect_factory_LIBS
  executor_LIBS
)

add_dependencies(${PROJECT_NAME}_node roborts_msgs_generate_messages)