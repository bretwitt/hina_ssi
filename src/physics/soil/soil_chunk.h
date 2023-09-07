#ifndef HINA_SSI_PLUGIN_SOIL_CHUNK_H
#define HINA_SSI_PLUGIN_SOIL_CHUNK_H

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/common/Profiler.hh>
#include <cmath>

#include "../../common/geometry.h"
#include "../../common/field/uniform_field.h"
#include "../../common/field/base_vertex_sampler.h"

#include "soil_vertex.h"
#include "soil_chunk_location.h"
#include "soil_vertex_sampler.h"

#include "footprint.h"
#include "triangle_context.h"

using ignition::math::Vector3d;
using ignition::math::Vector2d;

namespace hina {


    class SoilChunk {
    private:

        SoilChunkLocation location;

        std::shared_ptr<SoilVertexSampler> p_sampler = nullptr;
        std::shared_ptr <UniformField<SoilVertex>> p_field = nullptr;

    public:

        SoilChunk() = default;

        /*
         *  Initializes SoilChunk's field and vertex samplers
         */
        void init_chunk(FieldVertexDimensions dims, double scale, SoilChunkLocation location,
                        const std::shared_ptr<SoilVertexSampler>& sampler);

        /*
         *   Applies force to link and deforms chunk's graph based on Bekker-derived physics
         */
//        typedef std::vector<std::tuple<uint32_t, uint32_t, SoilChunk, std::shared_ptr<FieldVertex<SoilVertex>>>> Footprint_V;
        hina::Footprint try_deform(const TriangleContext& meshTri, const physics::LinkPtr& link);

        /*
         * Applies force to link and deforms chunk's graph based on Bekker-derived physics
         */
        void terramx_deform(const physics::LinkPtr &linkPtr, const TriangleContext &meshTri, uint32_t x, uint32_t y,
                            const std::shared_ptr<FieldVertex<SoilVertex>> &vertex, double w,
                            SoilPhysicsParams vert_attr);

        /*
         *  Calculate terramechanical forces based on a triangle-point contact
         */
        void terramx_contact(const SoilPhysicsParams& vert_attr,
                             const TriangleContext& contact_tri,
                             const std::shared_ptr<FieldVertex<SoilVertex>>& soil_vertex,
                             const double& abb_point_area,
                             Vector3d& contact_force,
                             double& sinkage);

        /*
         * Field getter
         */
        std::shared_ptr <UniformField<SoilVertex>> get_field() {
            return p_field;
        }

        /*
         * Location getter
         */
        SoilChunkLocation get_location() {
            return location;
        }

        /*
         * Reset chunk's vertex footprint data
         */
        void clear_footprint();

        /*
         *  Return true if a soil vertex penetrates a triangle
         */
        bool penetrates(const Triangle& meshTri, const std::shared_ptr<FieldVertex<SoilVertex>>& vtx, double w);

        /*
         * Return true if triangle overlaps AABB
         */
        static bool intersects_projected(const Triangle& meshTri, const AABB& vertexRect);

    };
}


#endif //HINA_SSI_PLUGIN_SOIL_CHUNK_H
