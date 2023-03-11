#ifndef SOIL_CPP
#define SOIL_CPP

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

#include "../../common/geometry.h"
#include "soil_data.h"
#include <cmath>
#include <gz/math/Vector3.hh>
#include "../dem/dem.h"
#include "../../../thirdparty/PerlinNoise.h"
#include "../../common/field/uniform_field.h"
#include "soil_chunk.h"
#include "../../common/field/chunked_field.h"
#include "../../common/field/base_vertex_sampler.h"
#include "../sandbox/sandbox_vertex_sampler.h"
#include "../dem/dem_vertex_sampler.h"

namespace hina {
    class Soil {

    private:
        FieldVertexDimensions vtx_dims{};
        std::shared_ptr<SoilVertexSampler> sampler = nullptr;

        double scale = 0;

        Soil(FieldVertexDimensions dims, double scale);
        Soil(FieldTrueDimensions dims, double scale);

        ChunkedField<std::shared_ptr<SoilChunk>> chunks;

    public:

        ChunkedField<std::shared_ptr<SoilChunk>> get_chunks();

        Soil(SandboxConfig config);
        Soil(const std::shared_ptr<DEM>& dem);
        Soil();


        void query_chunk(Vector3d pos);
        void pre_update();
        void post_update();

        std::shared_ptr<SoilChunk> OnChunkCreation(int i, int j);

        Vector2d worldpos_to_chunk_idx(Vector3d pos);
        Vector2d chunk_idx_to_worldpos(int i, int j);

        std::vector<std::tuple<uint32_t, uint32_t, SoilChunk, std::shared_ptr<FieldVertex<SoilAttributes>>>>
            try_deform(const Triangle &meshTri, const physics::LinkPtr &link, float& displaced_volume, float dt);

    };
}
#endif