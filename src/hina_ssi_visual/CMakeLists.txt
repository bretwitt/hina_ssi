add_library(${PROJECT_NAME}_visual SHARED hina_ssi_visual_plugin.cpp mesh_generator.cpp)
target_link_libraries(${PROJECT_NAME}_visual ${GAZEBO_LIBRARIES} ${PROJECT_NAME}_msgs ${PROJECT_NAME}_common)
add_dependencies(${PROJECT_NAME}_visual ${PROJECT_NAME}_msgs ${PROJECT_NAME}_common)
