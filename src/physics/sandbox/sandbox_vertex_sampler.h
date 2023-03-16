#ifndef HINA_SSI_FLATVERTEXLOADER_H
#define HINA_SSI_FLATVERTEXLOADER_H

#include "../soil/soil_physics_params.h"
#include "../soil/soil_vertex_sampler.h"

namespace hina {
    class SandboxVertexSampler : public SoilVertexSampler {

    private:
        double angle;
        SoilPhysicsParams params;

    public:
        SandboxVertexSampler(double angle, SoilPhysicsParams params) {
            this->angle = angle;
            this->params = params;
        }

        double get_z_at_index(double x, double y) override {
            return y * tan(angle);
        };

        SoilPhysicsParams get_params_at_index(double x, double y) override {
            return params;
        }
    };
}

#endif //HINA_SSI_FLATVERTEXLOADER_H
