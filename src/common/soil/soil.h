#ifndef SOIL_CPP
#define SOIL_CPP

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

#include "../geometry.h"
#include "soil_data.h"
#include "../dem/dem.h"

namespace gazebo {
    class Soil {
    private:

        SoilData* _data;

    public:

        explicit Soil(SandboxConfig config);

        ~Soil();

        SoilData* get_data();

        void generate_sandbox_geometry(SandboxConfig config);
        void generate_sandbox_soil_vertices(SandboxConfig config);
        void generate_indices() const;

        std::vector<std::tuple<uint32_t, uint32_t, VertexAttributes*>> try_deform(const Triangle& meshTri, const physics::LinkPtr& link, float dt, float& displaced_volume);
        bool penetrates(const Triangle& meshTri, VertexAttributes* vertex, double w);
        static bool intersects_projected(const Triangle& meshTri, const AABB& vertexRect);

        void terramx_deform(const physics::LinkPtr& linkPtr, const Triangle& meshTri, uint32_t x, uint32_t y, VertexAttributes* vertex, double w, float dt, float& displaced_volume);

        Soil();

        Soil(DEM* dem);

        void load_dem_geometry(DEM *dem) const;
    };
}

#endif