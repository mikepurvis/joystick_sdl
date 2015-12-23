cmake_minimum_required(VERSION 2.8.3)
project(joystick_sdl)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs)

include(FindPkgConfig)
pkg_search_module(SDL2 REQUIRED sdl2)

include_directories(
  ${SDL2_INCLUDE_DIRS}
  ${CATKIN_INCLUDE_DIRS}
)

add_executable(joystick_sdl_node src/joystick.cpp)
target_link_libraries(joystick_sdl_node ${SDL2_LIBRARIES} ${CATKIN_LIBRARIES})
