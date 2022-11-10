add_library(${PROJECT_NAME}_common SHARED geometry.h geometry.cpp soil/soil.cpp soil/sandbox_config.h dem/dem_loader.h)
target_link_libraries(${PROJECT_NAME}_common ${GAZEBO_LIBRARIES} ${PROJECT_NAME}_msgs ${LIB_OPENMP})
add_dependencies(${PROJECT_NAME}_common ${PROJECT_NAME}_msgs)
