cmake_minimum_required(VERSION 2.8.3)
project(joystick_sdl)

find_package(catkin REQUIRED COMPONENTS diagnostic_updater roscpp roslib sensor_msgs)

include(FindPkgConfig)
pkg_search_module(SDL2 REQUIRED sdl2)
pkg_check_modules(SDL204 sdl2>=2.0.4)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS diagnostic_updater roscpp sensor_msgs
  DEPENDS ${SDL2_LIBRARIES}
)

include_directories(
  include
  ${SDL2_INCLUDE_DIRS}
  ${CATKIN_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/joystick)
target_link_libraries(${PROJECT_NAME} ${SDL2_LIBRARIES} ${catkin_LIBRARIES})
if(SDL204_FOUND)
  set_property(TARGET ${PROJECT_NAME} PROPERTY COMPILE_DEFINITIONS SDL2_ENABLE_JOYSTICK_POWER_LEVEL)
endif()

add_executable(${PROJECT_NAME}_node src/joystick_node)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${SDL2_LIBRARIES} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY mappings
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if (CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)

  roslint_cpp()
  roslint_add_test()
endif()

