if (WIN32)
    ADD_DEFINITIONS(-D_USE_MATH_DEFINES)
    FIND_PACKAGE(Eigen3 CONFIG REQUIRED)
    INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
endif()

if(UNIX AND NOT APPLE)
    FIND_PACKAGE(Eigen3 REQUIRED)
    INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
    ADD_COMPILE_OPTIONS(-Werror=return-type -Wall -Wextra -Wmissing-declarations -Wredundant-decls -Woverloaded-virtual)
endif()

if(APPLE)
    INCLUDE_DIRECTORIES(
        /usr/local/include/
        /usr/local/include/eigen3
        # Most recent versions of brew install here
        /opt/homebrew/include
        /opt/homebrew/include/eigen3)
    ADD_COMPILE_OPTIONS(-Werror=return-type -Wall -Wextra -Wmissing-declarations -Wredundant-decls -Woverloaded-virtual)
    LINK_DIRECTORIES(
        /usr/local/lib/
        /opt/homebrew/lib
        )
endif()

set(COPPELIASIM_DRIVER_HEADERS
    ${COPPELIASIM_DRIVER_DIR}/include/coppeliasim_driver.hpp)

set(COPPELIASIM_DRIVER_SOURCES
    ${COPPELIASIM_DRIVER_DIR}/src/coppeliasim_driver.cpp)

include_directories(${COPPELIASIM_DRIVER_DIR}/include/)

add_library(coppeliasim_driver ${COPPELIASIM_DRIVER_HEADERS}
                               ${COPPELIASIM_DRIVER_SOURCES})

target_link_libraries(coppeliasim_driver
    dqrobotics
    dqrobotics-interface-coppeliasim)
