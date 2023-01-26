#ifndef SOIL_CPP
#define SOIL_CPP

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

#include "../../common/geometry.h"
#include "soil_data.h"
#include <cmath>
#include "../dem/dem.h"
#include "../../../thirdparty/PerlinNoise.h"
#include "../../common/field/uniform_field.h"
#include "soil_chunk.h"

namespace hina {
    class Soil {
    private:
        Soil(FieldVertexDimensions dims, double scale);
        Soil(FieldTrueDimensions dims, double scale);

        SoilChunk sc;

    public:

        explicit Soil(SandboxConfig config);

        Soil(const std::shared_ptr<DEM>& dem);

        SoilChunk get_chunk();

        void generate_sandbox_geometry(SandboxConfig config);

        void load_dem_geometry(const std::shared_ptr<DEM> &dem) const;

        std::vector<std::tuple<uint32_t, uint32_t, std::shared_ptr<FieldVertex<SoilAttributes>>>>
        try_deform(const Triangle &meshTri, const physics::LinkPtr &link, float dt, float &displaced_volume);

    };
}
#endif