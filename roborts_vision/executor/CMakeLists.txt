project(executor)

add_library(${PROJECT_NAME}_LIBS
  gimbal/gimbal_control.cpp
)

target_link_libraries(${PROJECT_NAME}_LIBS
  PUBLIC
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)