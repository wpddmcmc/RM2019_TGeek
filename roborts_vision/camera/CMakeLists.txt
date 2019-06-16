project(camera)

add_library(${PROJECT_NAME}_LIBS
  mercure/mercure_driver.cpp
  uvc/RMVideoCapture.cpp
)

target_link_libraries(${PROJECT_NAME}_LIBS
  PUBLIC
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  gxiapi
)
