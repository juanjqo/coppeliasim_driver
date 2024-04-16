cmake_minimum_required(VERSION 3.5)

project(test_lib LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(COPPELIASIM_DRIVER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../coppeliasim_driver/)
include(${COPPELIASIM_DRIVER_DIR}/coppeliasim_driver_library.cmake)


add_executable(test_lib main.cpp)

target_link_libraries(test_lib
                      dqrobotics
                      coppeliasim_driver)
