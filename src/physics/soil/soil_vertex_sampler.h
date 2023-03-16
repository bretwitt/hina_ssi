#ifndef HINA_SSI_SOIL_VERTEX_SAMPLER_H
#define HINA_SSI_SOIL_VERTEX_SAMPLER_H

#include "soil_physics_params.h"
#include "../../common/field/base_vertex_sampler.h"

class SoilVertexSampler : public BaseVertexSampler {

public:
    virtual SoilPhysicsParams get_params_at_index(double x, double y) {
        return { 0, 0, 0, 0, 0};
    };
};

#endif //HINA_SSI_SOIL_VERTEX_SAMPLER_H
