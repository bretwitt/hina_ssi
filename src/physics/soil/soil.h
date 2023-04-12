#ifndef SOIL_CPP
#define SOIL_CPP

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

#include <cmath>
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

        std::shared_ptr<ChunkedField<std::shared_ptr<SoilChunk>>> chunks = nullptr;
        std::shared_ptr<SoilVertexSampler> sampler = nullptr;

    public:

        Soil(std::shared_ptr<SoilVertexSampler> sampler, FieldTrueDimensions dims, double scale);

        /*
         *  Get chunk
         */
        std::shared_ptr<ChunkedField<std::shared_ptr<SoilChunk>>> get_chunks();


        /*
         *  Load chunk or de-schedule from culling, takes world coordinates
         */
        void query_chunk(const Vector3d& pos);

        /*
         *  Prepare Soil for update step
         */
        void pre_update();

        /*
         *  Prepare Soil for next update cycle
         */
        void post_update();

        /*
         * Register callback that gets called when any SoilChunk is loaded
         */
        std::shared_ptr<SoilChunk> OnChunkCreation(int i, int j);

        /*
         * Get chunk index from world position
         */
        Vector2d worldpos_to_chunk_idx(Vector3d pos) const;

        /*
         * Get world position of chunk edge from chunk index
         */
        Vector2d chunk_idx_to_worldpos(int i, int j) const;

        /*
         *      Get vertex from global vertex index
         */
        std::shared_ptr<FieldVertex<SoilVertex>> get_vertex_from_global(int g_x, int g_y);

        /*
         *      Get global vertex index from vertex index in chunk frame
         */
        void get_global_idx(SoilChunkLocation loc, int x, int y, int& g_x, int& g_y);

        /*
          *      Get global vertex index from world position
          */
        void get_nearest_gidx(Vector3d pos, int& g_x, int& g_y);

        /*
         *  Applies force to link and deforms appropriate chunk's graph based on Bekker-derived physics
         */
        typedef std::vector<std::tuple<uint32_t, uint32_t,SoilChunk&, std::shared_ptr<FieldVertex<SoilVertex>>>> Field_V;
        hina::Soil::Field_V try_deform(const Triangle &meshTri, const physics::LinkPtr &link, double &displaced_volume);

        /*
         *  Perform footprint level computations
         */
        typedef std::vector<std::vector<std::tuple<uint32_t, uint32_t, SoilChunk& ,std::shared_ptr<FieldVertex<SoilVertex>>>>> Footprint_V;
        void compute_footprint_stage(const Footprint_V& footprint, double total_displaced_volume);


        /*
         *  Compute deformation on graph at certain chunk
         */
        hina::Soil::Field_V deform_chunk(const std::shared_ptr<SoilChunk>& chunk, const Triangle& meshTri,
                                         const physics::LinkPtr& link,double& displaced_vol);

    };
}
#endif