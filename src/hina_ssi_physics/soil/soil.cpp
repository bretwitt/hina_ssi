#include "soil.h"

using namespace gazebo;
using namespace hina;

using ignition::math::Vector3d;

Soil::Soil(FieldVertexDimensions dims, double scale) : Soil() {
    this->scale = scale;
    vtx_dims = dims;
}

Soil::Soil(FieldTrueDimensions dims, double scale) : Soil() {
    this->scale = scale;
    vtx_dims = UniformField<SoilAttributes>::as_vtx_dims(dims,scale);
}

Soil::Soil() {
    chunks.register_chunk_create_callback(boost::bind(&Soil::OnChunkCreation, this, _1, _2));
}

Soil::Soil(SandboxConfig config) : Soil(FieldTrueDimensions { config.x_width, config.y_width }, config.scale) {
}

Soil::Soil(const std::shared_ptr<DEM>& dem) : Soil(FieldVertexDimensions { dem->field->y_vert_width, dem->field->x_vert_width }, dem->field->scale) {
    load_dem_geometry(dem);
}

void Soil::generate_sandbox_geometry(SandboxConfig config) {
}

void Soil::load_dem_geometry(const std::shared_ptr<DEM>& dem) const {
//    auto field = sc->field;
//    for(int i = 0; i < dem->field->y_vert_width; i++) {
//        for(int j = 0; j < dem->field->x_vert_width; j++) {
//            Vector3d dem_v3 = dem->field->get_vertex_at_index(i,j)->v3;
//            Vector3d v3 ( dem_v3.X(), dem_v3.Y(), dem_v3.Z());
//            field->set_vertex_at_index(i, j, std::make_shared<FieldVertex<SoilAttributes>>(v3));
//        }
//    }
}

std::vector<std::tuple<uint32_t, uint32_t, std::shared_ptr<FieldVertex<SoilAttributes>>>> Soil::try_deform(const Triangle& meshTri, const physics::LinkPtr& link, float dt, float& displaced_volume) {
    auto idx = worldpos_to_chunk_idx(meshTri.centroid());
    auto chunk = chunks.get_chunk_cont({idx.X(),idx.Y()});
    if (chunk != nullptr) {
        return chunk->try_deform(meshTri, link, dt);
    }
}

Vector2d Soil::chunk_idx_to_worldpos(int i, int j) {
    return Vector2d( i*(this->vtx_dims.verts_x*this->scale), j*(this->vtx_dims.verts_y*this->scale) );
}

Vector2d Soil::worldpos_to_chunk_idx(Vector3d pos) {
    return Vector2d( floor(pos.X() / (this->vtx_dims.verts_x*this->scale)), floor(pos.Y() / (this->vtx_dims.verts_y*this->scale)) );
}

void Soil::query_chunk(Vector3d pos) {
    auto v2 = worldpos_to_chunk_idx(pos);
    auto i = v2.X();
    auto j = v2.Y();

    // TODO: RADIUS
    chunks.poll_chunk({i,j});
    chunks.poll_chunk({i + 1,j});
    chunks.poll_chunk({i - 1,j});
    chunks.poll_chunk({i,j + 1});
    chunks.poll_chunk({i,j - 1});
}

void Soil::start_chunk_poll() {
    chunks.pre_update();
}

std::shared_ptr<SoilChunk> Soil::OnChunkCreation(int i, int j) {
    auto sc = std::make_shared<SoilChunk>();
    sc->init_chunk(vtx_dims, scale, { i,j, chunk_idx_to_worldpos(i,j)});
    return sc;
}

void Soil::unload_dead_chunks() {
    chunks.post_update();
}
