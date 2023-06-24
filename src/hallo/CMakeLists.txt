cmake_minimum_required(VERSION 3.0.2)
project(hallo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  cv_bridge
  message_filters
)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(depth src/hello_c.cpp)
target_link_libraries(depth
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

install(TARGETS depth
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
