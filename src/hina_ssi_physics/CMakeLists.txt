    add_library(${PROJECT_NAME}_physics SHARED
                        hina_ssi_physics_plugin.cpp
                        dem/dem.h dem/dem_loader.h
                        soil/soil.cpp soil/soil.h
                        soil/soil_data.h
                        soil/sandbox_config.h
            )
    target_link_libraries(${PROJECT_NAME}_physics PUBLIC ${GAZEBO_LIBRARIES} ${PROJECT_NAME}_msgs ${PROJECT_NAME}_common ${LIB_OPENMP} )
    add_dependencies(${PROJECT_NAME}_physics ${PROJECT_NAME}_msgs ${PROJECT_NAME}_common)
