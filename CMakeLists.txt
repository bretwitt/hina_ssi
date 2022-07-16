cmake_minimum_required(VERSION 3.16)
project(hina_ssi_plugin)

find_package(gazebo REQUIRED)
find_package(CGAL REQUIRED)

include_directories(include ${GAZEBO_INCLUDE_DIRS} ${CMAKE_CURRENT_BINARY_DIR}/src/msgs)
link_directories(${GAZEBO_LIBRARY_DIRS} ${CMAKE_CURRENT_BINARY_DIR}/src/msgs)

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_subdirectory(src/msgs)
add_subdirectory(src/hina_ssi_visual)
add_subdirectory(src/hina_ssi_physics)