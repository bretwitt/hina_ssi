#ifndef HINA_SSI_DEM_VERTEX_LOADER_H
#define HINA_SSI_DEM_VERTEX_LOADER_H
#include "../../common/field/base_vertex_sampler.h"
#include "dem.h"
#include "../soil/soil_vertex_sampler.h"

namespace hina {
    class DEMVertexSampler : public SoilVertexSampler {

    private:
        std::shared_ptr<DEM> dem;

    public:
        DEMVertexSampler(const std::shared_ptr<DEM>& dem) {
            this->dem = dem;
        }

        SoilPhysicsParams get_params_at_index(double x, double y) override {
            return { 814000.0f, 7.8e7, 20680.0f, 3500, 0.55, 0.25 };
        }

        double get_z_at_index(double x, double y) override {
            //return 0;
            return dem->get_z_at_position(x,y);
        }
    };
}

#endif //HINA_SSI_DEM_VERTEX_LOADER_H
