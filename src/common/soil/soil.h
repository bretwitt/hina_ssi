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
        std::shared_ptr<SoilData> _data;
        Soil(uint32_t width, uint32_t height, double scale);

    public:

        explicit Soil(SandboxConfig config);
        Soil(std::shared_ptr<DEM> dem);


        std::shared_ptr<SoilData> get_data();

        void generate_sandbox_geometry(SandboxConfig config);
        void generate_sandbox_soil_vertices(SandboxConfig config);
        void generate_indices() const;
        void load_dem_geometry(const std::shared_ptr<DEM>& dem) const;

        std::vector<std::tuple<uint32_t, uint32_t, std::shared_ptr<VertexAttributes>>> try_deform(const Triangle& meshTri, const physics::LinkPtr& link, float dt, float& displaced_volume);
        bool penetrates(const Triangle& meshTri, const std::shared_ptr<VertexAttributes>& vertex, double w);
        static bool intersects_projected(const Triangle& meshTri, const AABB& vertexRect);

        void terramx_deform(const physics::LinkPtr& linkPtr, const Triangle& meshTri, uint32_t x, uint32_t y, const std::shared_ptr<VertexAttributes>& vertex, double w, float dt, float& displaced_volume);


    };
}

#endif