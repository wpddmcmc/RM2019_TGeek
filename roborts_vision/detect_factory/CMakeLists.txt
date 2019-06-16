project(detect_factory)

add_library(${PROJECT_NAME}_LIBS
  armor/constraint_set.cpp
)

target_link_libraries(${PROJECT_NAME}_LIBS
  PUBLIC
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)