cmake_minimum_required(VERSION 3.16)
project(hina_ssi_plugin)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(LIB_OPENMP "")

find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    message(STATUS "Using OpenMP ")
    list(APPEND LIB_OPENMP OpenMP::OpenMP_CXX)
endif()

set(CMAKE_CXX_STANDARD 17)

find_package(gazebo REQUIRED)

include_directories(include ${GAZEBO_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR}/src/msgs)
link_directories(${GAZEBO_LIBRARY_DIRS} ${CMAKE_CURRENT_BINARY_DIR}/src/msgs)

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_subdirectory(src/msgs)
add_subdirectory(src/common)
add_subdirectory(src/visual)
add_subdirectory(src/physics)
