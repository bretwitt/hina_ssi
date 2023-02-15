find_package(libInterpolate REQUIRED)

add_library(${PROJECT_NAME}_physics SHARED
                hina_ssi_physics_plugin.cpp
                dem/dem.h dem/dem_loader.h
                soil/soil.cpp soil/soil.h
                soil/soil_data.h
                soil/sandbox_config.h
                soil/soil_chunk.h
                soil/soil_chunk_location_metadata.h
        )

target_link_libraries(${PROJECT_NAME}_physics PUBLIC ${GAZEBO_LIBRARIES} ${PROJECT_NAME}_msgs ${PROJECT_NAME}_common ${LIB_OPENMP} libInterpolate::Interpolate)
add_dependencies(${PROJECT_NAME}_physics ${PROJECT_NAME}_msgs ${PROJECT_NAME}_common)
