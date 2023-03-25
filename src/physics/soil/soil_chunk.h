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

using ignition::math::Vector3d;
using ignition::math::Vector2d;

namespace hina {


    class SoilChunk {
    private:

        Vector2d min;

        std::shared_ptr<SoilVertexSampler> sampler = nullptr;

        SoilChunkLocationMetadata location;
        std::shared_ptr <UniformField<SoilVertex>> field = nullptr;

    public:
        Vector2d max;

        SoilChunk() = default;

        /*
         *  Initializes SoilChunk's field and vertex samplers
         */
        void init_chunk(FieldVertexDimensions dims, double scale, SoilChunkLocationMetadata location,
                        const std::shared_ptr<SoilVertexSampler>& sampler);

        /*
         * Field getter
         */
        std::shared_ptr <UniformField<SoilVertex>> get_field() {
            return field;
        }

        /*
         * Location getter
         */
        SoilChunkLocationMetadata get_location() {
            return location;
        }

        /*
         *   Applies force to link and deforms chunk's graph based on Bekker-derived physics
         */
        typedef std::vector<std::tuple<uint32_t, uint32_t, SoilChunk, std::shared_ptr<FieldVertex<SoilVertex>>>> Footprint_V;
        Footprint_V try_deform(const Triangle& meshTri, const physics::LinkPtr& link, double& displaced_vol);

        /*
         * Applies force to link and deforms chunk's graph based on Bekker-derived physics
         */
        void terramx_deform(const physics::LinkPtr &linkPtr, const Triangle &meshTri, uint32_t x, uint32_t y,
                            const std::shared_ptr<FieldVertex<SoilVertex>> &vertex, double w,
                            double &displaced_volume, SoilPhysicsParams vert_attr);

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
