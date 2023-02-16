#include "soil.h"

using namespace gazebo;
using namespace hina;

using ignition::math::Vector3d;


Soil::Soil(SandboxConfig config) : Soil(FieldTrueDimensions { config.x_width, config.y_width }, config.scale) {
    sampler = std::make_shared<SandboxVertexSampler>(config.angle);
}

Soil::Soil(FieldVertexDimensions dims, double scale) : Soil() {
    this->scale = scale;
    vtx_dims = dims;
}

Soil::Soil(FieldTrueDimensions dims, double scale) : Soil() {
    this->scale = scale;
    vtx_dims = UniformField<SoilAttributes>::as_vtx_dims(dims,scale);
}

Soil::Soil(const std::shared_ptr<DEM>& dem) : Soil(FieldVertexDimensions { dem->field->y_vert_width, dem->field->x_vert_width }, dem->field->scale) {
}

Soil::Soil() {
    chunks.register_chunk_create_callback(boost::bind(&Soil::OnChunkCreation, this, _1, _2));
}

std::vector<std::tuple<uint32_t, uint32_t, std::shared_ptr<FieldVertex<SoilAttributes>>>> Soil::try_deform(const Triangle& meshTri, const physics::LinkPtr& link, float dt, float& displaced_volume) {
    auto idx = worldpos_to_chunk_idx(meshTri.centroid());
    auto chunk = chunks.get_chunk_cont({idx.X(),idx.Y()});
    if (chunk != nullptr) {
        return chunk->try_deform(meshTri, link, dt);
    }
}

Vector2d Soil::chunk_idx_to_worldpos(int i, int j) {
    return Vector2d( i*((this->vtx_dims.verts_x-1)*this->scale), j*((this->vtx_dims.verts_y-1)*this->scale) );
}

Vector2d Soil::worldpos_to_chunk_idx(Vector3d pos) {
    return Vector2d( floor(pos.X() / (this->vtx_dims.verts_x*this->scale)), floor(pos.Y() / (this->vtx_dims.verts_y*this->scale)) );
}

void Soil::query_chunk(Vector3d pos) {
    auto v2 = worldpos_to_chunk_idx(pos);
    auto i = static_cast<int>(v2.X());
    auto j = static_cast<int>(v2.Y());

    // TODO: RADIUS
    chunks.poll_chunk({i,j});
    chunks.poll_chunk({i + 1,j});
    chunks.poll_chunk({i - 1,j});
    chunks.poll_chunk({i,j + 1});
    chunks.poll_chunk({i,j - 1});
}

void Soil::pre_update() {
    chunks.pre_update();
}

std::shared_ptr<SoilChunk> Soil::OnChunkCreation(int i, int j) {
    auto sc = std::make_shared<SoilChunk>();
    sc->init_chunk(vtx_dims, scale, { i,j, chunk_idx_to_worldpos(i,j)}, sampler);
    return sc;
}

void Soil::post_update() {
    chunks.post_update();
}
