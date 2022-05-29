cmake_minimum_required(VERSION 3.16)
project(hina_ssi_plugin)

find_package(gazebo REQUIRED)
include_directories(include ${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(${PROJECT_NAME} SHARED src/hina_ssi_plugin.cpp)
target_link_libraries(${PROJECT_NAME} ${GAZEBO_LIBRARIES})
