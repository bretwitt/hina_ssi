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
#include "../../common/field/chunked_field.h"

namespace hina {
    class Soil {

    private:
        Soil(FieldVertexDimensions dims, double scale);
        Soil(FieldTrueDimensions dims, double scale);

        FieldVertexDimensions vtx_dims;

        double scale = 0;

//        std::unordered_map<uint32_t,std::unordered_map<uint32_t,std::shared_ptr<SoilChunk>>> chunk_map;
        ChunkedField<std::shared_ptr<SoilChunk>> chunks;

    public:
        std::vector<std::shared_ptr<SoilChunk>> active_chunks;

        Soil(SandboxConfig config);

        Soil(const std::shared_ptr<DEM>& dem);

        Soil();

        void generate_sandbox_geometry(SandboxConfig config);

        void load_dem_geometry(const std::shared_ptr<DEM> &dem) const;

        void query_chunk(Vector3d pos);
        void start_chunk_poll();
        void unload_dead_chunks();

        std::shared_ptr<SoilChunk> OnChunkCreation(int i, int j);

        Vector2d worldpos_to_chunk_idx(Vector3d pos);
        Vector2d chunk_idx_to_worldpos(int i, int j);

        std::vector<std::tuple<uint32_t, uint32_t, std::shared_ptr<FieldVertex<SoilAttributes>>>>
        try_deform(const Triangle &meshTri, const physics::LinkPtr &link, float dt, float &displaced_volume);

    };
}
#endif