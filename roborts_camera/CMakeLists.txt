project(roborts_camera)

set(CMAKE_CXX_STANDARD 14)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  cv_bridge
  image_transport
  
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS
)

include_directories(
  ${OpenCV_INCLUDE_DIRECTORIES}
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(mercure)

add_executable(${PROJECT_NAME}_node
  test/main.cpp 
)

target_link_libraries(${PROJECT_NAME}_node
  PRIVATE
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  mercure_driver
)
