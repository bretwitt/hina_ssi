#ifndef SOIL_CPP
#define SOIL_CPP

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

#include "geometry.h"
#include "soil_data.h"

namespace gazebo {
    class Soil {
    private:
        SoilData* _data;
        double sigma_yield = 0;

    public:

        explicit Soil(SoilData* soil_data);

        ~Soil();

        SoilData* get_data();

        void generate_geometry();
        void generate_soil_vertices();
        void generate_indices() const;

        void pre_update();
        void try_deform(const Triangle& meshTri, const physics::LinkPtr& link, float dt);
        bool penetrates(const Triangle& meshTri, VertexAttributes* vertex, double w);
        static bool intersects_projected(const Triangle& meshTri, const AABB& vertexRect);

        void terramx_deform(const physics::LinkPtr& linkPtr, const Triangle& meshTri, uint32_t x, uint32_t y, VertexAttributes* vertex, double w, float dt);
    };
}

#endif