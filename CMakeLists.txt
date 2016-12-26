cmake_minimum_required(VERSION 2.8.3)
project(minip)
find_package(catkin REQUIRED COMPONENTS
  roscpp
   move_base_msgs
  tf
)
find_package( OpenCV REQUIRED )


catkin_package(
)

include_directories( ${OpenCV_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(minip src/minip.cpp)
target_link_libraries(minip ${catkin_LIBRARIES} ${OpenCV_LIBS})


target_link_libraries(minip
  ${catkin_LIBRARIES}
)
