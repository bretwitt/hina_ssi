#ifndef HINA_SSI_FLATVERTEXLOADER_H
#define HINA_SSI_FLATVERTEXLOADER_H

#include "../../common/field/base_vertex_sampler.h"
#include "../soil/soil_physics_params.h"
#include "../soil/soil_vertex_sampler.h"


class SandboxVertexSampler : public SoilVertexSampler {

private:
    double angle;
    SoilPhysicsParams params;

public:
    SandboxVertexSampler(double angle, SoilPhysicsParams params) {
        this->angle = angle;
        this->params = params;
    }

    double get_z_at_index(double x, double y) {
        return y*tan(angle);
    };

    SoilPhysicsParams get_params_at_index(double x, double y) {
        return params;
    }
};

#endif //HINA_SSI_FLATVERTEXLOADER_H
