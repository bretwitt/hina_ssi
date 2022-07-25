add_library(${PROJECT_NAME}_visual SHARED hina_ssi_visual_plugin.cpp mesh_generator.cpp ../common/soil.cpp ../common/geometry.cpp)
target_link_libraries(${PROJECT_NAME}_visual ${GAZEBO_LIBRARIES} ${PROJECT_NAME}_msgs)
add_dependencies(${PROJECT_NAME}_visual ${PROJECT_NAME}_msgs)
