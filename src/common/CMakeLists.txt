add_library(${PROJECT_NAME}_common SHARED geometry.h geometry.cpp soil.cpp)
target_link_libraries(${PROJECT_NAME}_common ${GAZEBO_LIBRARIES} ${PROJECT_NAME}_msgs)
add_dependencies(${PROJECT_NAME}_common ${PROJECT_NAME}_msgs)
