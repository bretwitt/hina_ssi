    add_library(${PROJECT_NAME}_physics SHARED hina_ssi_physics_plugin.cpp)
    target_link_libraries(${PROJECT_NAME}_physics ${GAZEBO_LIBRARIES} ${PROJECT_NAME}_msgs)
    add_dependencies(${PROJECT_NAME}_physics ${PROJECT_NAME}_msgs)
