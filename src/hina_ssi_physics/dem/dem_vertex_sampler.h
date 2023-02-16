#ifndef HINA_SSI_DEM_VERTEX_LOADER_H
#define HINA_SSI_DEM_VERTEX_LOADER_H
#include "../../common/field/base_vertex_sampler.h"

class DEMVertexLoader : BaseVertexSampler {

    DEMVertexLoader() {

    }

    double get_z_at_index(Vector2d pos) override {

    }
};

#endif //HINA_SSI_DEM_VERTEX_LOADER_H
