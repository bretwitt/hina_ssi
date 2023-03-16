#ifndef SOIL_CPP
#define SOIL_CPP

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

#include <cmath>
#include <gz/math/Vector3.hh>
#include "../../../thirdparty/PerlinNoise.h"

#include "soil_chunk.h"
#include "soil_vertex.h"

#include "../../common/field/field.h"
#include "../../common/geometry.h"

#include "../sandbox/sandbox_vertex_sampler.h"
#include "../dem/dem_vertex_sampler.h"
#include "../dem/dem.h"


namespace hina {
    class Soil {

    private:

        FieldVertexDimensions vtx_dims{};
        double scale = 0;

        Soil(FieldVertexDimensions dims, double scale);
        Soil(FieldTrueDimensions dims, double scale);
        Soil();

        std::shared_ptr<ChunkedField<std::shared_ptr<SoilChunk>>> chunks;
        std::shared_ptr<SoilVertexSampler> sampler = nullptr;

    public:

        Soil(std::shared_ptr<SoilVertexSampler> sampler, FieldTrueDimensions dims, double scale);

        std::shared_ptr<ChunkedField<std::shared_ptr<SoilChunk>>> get_chunks();




        /*
         *  Load chunk or de-schedule from culling
         */
        void query_chunk(const Vector3d& pos);

        /*
         *  Prepare Soil for update step
         */
        void pre_update();

        /*
         *  Prepare Soil for next cycle of updates
         */
        void post_update();

        /*
         * Registered callback that gets called when any SoilChunk is loaded
         */
        std::shared_ptr<SoilChunk> OnChunkCreation(int i, int j);

        /*
         *
         */
        Vector2d worldpos_to_chunk_idx(Vector3d pos) const;

        /*
         *
         */
        Vector2d chunk_idx_to_worldpos(int i, int j) const;

        /*
         *  Applies force to link and deforms appropriate chunk's graph based on Bekker-derived physics
         */
        typedef std::vector<std::tuple<uint32_t, uint32_t,SoilChunk, std::shared_ptr<FieldVertex<SoilVertex>>>> Field_V;
        Field_V try_deform(const Triangle &meshTri, const physics::LinkPtr &link, double& displaced_volume, double dt);

    };
}
#endif