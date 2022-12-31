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

        void generate_sandbox_geometry(SandboxConfig config);

        void generate_sandbox_soil_vertices(SandboxConfig config);

        void load_dem_geometry(const std::shared_ptr<DEM> &dem) const;

        std::vector<std::tuple<uint32_t, uint32_t, std::shared_ptr<FieldVertex<SoilAttributes>>>>
        try_deform(const Triangle &meshTri, const physics::LinkPtr &link, float dt, float &displaced_volume);

        static bool intersects_projected(const Triangle &meshTri, const AABB &vertexRect);

        std::shared_ptr<UniformField<SoilAttributes>> field;

        bool penetrates(const Triangle &meshTri, const std::shared_ptr<FieldVertex<SoilAttributes>> &vtx, double w);

        void terramx_deform(const physics::LinkPtr &linkPtr, const Triangle &meshTri, uint32_t x, uint32_t y,
                            const std::shared_ptr<FieldVertex<SoilAttributes>> &vertex, double w, float dt,
                            float &displaced_volume);
    };
}
#endif