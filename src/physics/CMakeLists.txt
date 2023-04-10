find_package(libInterpolate REQUIRED)
find_package(ignition-common3 REQUIRED COMPONENTS profiler)

add_library(${PROJECT_NAME}_physics SHARED
                hina_ssi_physics_plugin.cpp
                dem/dem.h dem/dem_loader.h
                soil/soil.cpp soil/soil.h
                soil/soil_vertex.h
                soil/soil_chunk.h soil/soil_chunk.cpp
                soil/soil_chunk_location.h
                scm/terramechanics.h scm/terramechanics.cpp
        )

target_link_libraries(${PROJECT_NAME}_physics PUBLIC ${GAZEBO_LIBRARIES} ${PROJECT_NAME}_msgs ${PROJECT_NAME}_common ${LIB_OPENMP} libInterpolate::Interpolate ignition-common3::profiler)
add_dependencies(${PROJECT_NAME}_physics ${PROJECT_NAME}_msgs ${PROJECT_NAME}_common)
#target_compile_definitions(${PROJECT_NAME}_physics PUBLIC "IGN_PROFILER_ENABLE=1")
