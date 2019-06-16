project(mercure_driver)

add_library(${PROJECT_NAME}
    mercure_driver.cpp
)

target_link_libraries(${PROJECT_NAME}
  PUBLIC
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  gxiapi
  pthread
)
