cmake_minimum_required(VERSION 2.8.3)
project(extract_sync_images)

add_compile_options(-std=c++11)

find_package(OpenCV) 
find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_filters
  sensor_msgs
  std_msgs
  cv_bridge
  nav_msgs
)

catkin_package(CATKIN_DEPENDS)
include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(extract_sync_images src/extract_sync_images.cpp)
target_link_libraries(extract_sync_images ${catkin_LIBRARIES})

add_executable(test src/test.cpp)
target_link_libraries(test ${catkin_LIBRARIES})
