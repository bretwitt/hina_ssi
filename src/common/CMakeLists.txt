find_package(libInterpolate REQUIRED)

add_library(${PROJECT_NAME}_common SHARED field/uniform_field.h field/field_vertex.h geometry.cpp geometry.h field/vertex_dims.h)
target_link_libraries(${PROJECT_NAME}_common ${GAZEBO_LIBRARIES} ${PROJECT_NAME}_msgs ${LIB_OPENMP} )
add_dependencies(${PROJECT_NAME}_common ${PROJECT_NAME}_msgs)
