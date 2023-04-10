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
\
        /*
         * Field getter
         */
        std::shared_ptr <UniformField<SoilVertex>> get_field() {
            return p_field;
        }

        /*
         * Sampler getter
         */
        std::shared_ptr<SoilVertexSampler> get_sampler() {
            return p_sampler;
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

    };
}


#endif //HINA_SSI_PLUGIN_SOIL_CHUNK_H
