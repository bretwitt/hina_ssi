#ifndef HINA_SSI_PLUGIN_DEM
#define HINA_SSI_PLUGIN_DEM

#include "../soil/soil.h"
#include <cmath>
#include <omp.h>

using namespace gazebo;

class DEM {

public:
    std::shared_ptr<UniformField<SoilAttributes>> field = nullptr;

    DEM(uint32_t width, uint32_t length, double scale) {
          field = std::make_shared<UniformField<SoilAttributes>>(width, length, scale);
          field->init_field();
          field->generate_vertices();
    }

    void load_vertex(uint32_t row, uint32_t col, double z) {
        auto vert = field->get_vertex_at_index(row, col);
        Vector3d v3(vert->v3.X(), vert->v3.Y(), z);
        vert->v3 = v3;
        vert->v3_0 = v3;
    }

    void load_vertex(int flattened_index, double z) {
        auto vert = field->get_vertex_at_flattened_index(flattened_index);
        Vector3d v3(vert->v3.X(), vert->v3.Y(), z);
        vert->v3 = v3;
        vert->v3_0 = v3;
    }


};

#endif

