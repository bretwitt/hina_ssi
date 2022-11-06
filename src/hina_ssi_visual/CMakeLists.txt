add_library(${PROJECT_NAME}_visual SHARED hina_ssi_visual_plugin.cpp ogre_soil_renderer.cpp render_config.cpp render_config.h)
target_link_libraries(${PROJECT_NAME}_visual ${GAZEBO_LIBRARIES} ${PROJECT_NAME}_msgs ${PROJECT_NAME}_common ${LIB_OPENMP} )
add_dependencies(${PROJECT_NAME}_visual ${PROJECT_NAME}_msgs ${PROJECT_NAME}_common)
