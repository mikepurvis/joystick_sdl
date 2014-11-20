cmake_minimum_required(VERSION 2.8.3)
project(joystick_sdl)

include(FindSDL)
find_package(SDL REQUIRED)

include_directories(
  ${SDL_INCLUDE_DIR}
)

add_executable(joystick_sdl_node src/joystick.cpp)
target_link_libraries(joystick_sdl_node ${SDL_LIBRARY})

