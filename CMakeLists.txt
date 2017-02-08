cmake_minimum_required(VERSION 2.8.3)
project(kuka_record_and_play)

find_package(catkin REQUIRED COMPONENTS
  lwr_controllers
  roscpp
)
add_definitions (-fpermissive -std=c++11)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(record src/record.cpp)
add_dependencies(record ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(record
  ${catkin_LIBRARIES}
)


add_executable(play src/play.cpp)
add_dependencies(play ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(play
  ${catkin_LIBRARIES}
)
