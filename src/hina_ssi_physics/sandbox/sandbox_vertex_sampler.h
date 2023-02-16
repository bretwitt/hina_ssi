#ifndef HINA_SSI_FLATVERTEXLOADER_H
#define HINA_SSI_FLATVERTEXLOADER_H

#include "../../common/field/base_vertex_sampler.h"

class SandboxVertexSampler : public BaseVertexSampler {

private:
    double angle;
public:
    SandboxVertexSampler(double angle) {
        this->angle = angle;
        name = "sandbox";
    }

    double get_z_at_index(double x, double y) {
        return y*tan(angle);
    };
};

#endif //HINA_SSI_FLATVERTEXLOADER_H
