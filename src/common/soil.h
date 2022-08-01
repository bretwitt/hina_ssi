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

    public:

        explicit Soil(SoilData* soil_data);

        ~Soil();

        SoilData* get_data();

        void generate_soil_vertices();
        void generate_indices() const;
        void try_deform(const Triangle& meshTri, const physics::LinkPtr& link, float dt);
        void footprint_hash_idx_lookup_and_terramx_deform(const Triangle& meshTri, const physics::LinkPtr& link, float dt);
        bool penetrates(const Triangle& meshTri, const Vector3d& point, double w);
        bool intersects_projected(const Triangle& meshTri, const AABB& vertexRect);
        void terramx_deform(const physics::LinkPtr& linkPtr, const Triangle& meshTri, uint32_t x, uint32_t y, Vector3d v3, double w, float dt);
        Vector3d tri_normal(const Triangle& tri);
        void apply_normal_force(const physics::LinkPtr& linkPtr, const Vector3d& origin, const Vector3d& normal_force, float dt);
    };
}

#endif