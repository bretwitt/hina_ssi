#ifndef HINA_SSI_DEM_VERTEX_LOADER_H
#define HINA_SSI_DEM_VERTEX_LOADER_H
#include "../../common/field/base_vertex_sampler.h"
#include "dem.h"

namespace hina {
    class DEMVertexSampler : public BaseVertexSampler {

    private:
        std::shared_ptr<DEM> dem;

    public:
        DEMVertexSampler(const std::shared_ptr<DEM>& dem) : DEMVertexSampler() {
            this->dem = dem;
        }

        DEMVertexSampler() {

        }

        double get_z_at_index(double x, double y) {
            return dem->get_z_at_position(x,y);
        }
    };
}

#endif //HINA_SSI_DEM_VERTEX_LOADER_H
