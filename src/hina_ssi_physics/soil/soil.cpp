#include "soil.h"

using namespace gazebo;
using namespace hina;

using ignition::math::Vector3d;

Soil::Soil(FieldVertexDimensions dims, double scale) {
    this->scale = scale;
//    this->sc = std::make_shared<SoilChunk>();
//    this->sc->init_chunk(dims, scale, {0,0, Vector2d(0,0)});
    vtx_dims = dims;
}

Soil::Soil(FieldTrueDimensions dims, double scale) {
    this->scale = scale;
//    this->sc = std::make_shared<SoilChunk>();
//    this->sc->init_chunk(dims, scale, {0,0, Vector2d(0,0)});
    vtx_dims = UniformField<SoilAttributes>::as_vtx_dims(dims,scale);
}

Soil::Soil(SandboxConfig config) : Soil(FieldTrueDimensions { config.x_width, config.y_width }, config.scale) {
}

Soil::Soil(const std::shared_ptr<DEM>& dem) : Soil(FieldVertexDimensions { dem->field->y_vert_width, dem->field->x_vert_width }, dem->field->scale) {
    load_dem_geometry(dem);
}

void Soil::generate_sandbox_geometry(SandboxConfig config) {
    //this->sc->generate_vertices(config);
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
    auto chunk = chunk_map[idx.X()][idx.Y()];
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

    auto sc = chunk_map[i][j];

    if(sc == nullptr) {
        load_chunk(i,j);
    } else {
        sc->mark_unload(false);
    }
}

void Soil::start_chunk_poll() {
    for(auto c : active_chunks) {
        c->mark_unload(true);
    }
}

void Soil::unload_dead_chunks() {
    for(int k = 0; k < active_chunks.size(); k++) {
        if(active_chunks[k]->unload_flag) {
            auto id = active_chunks[k]->location;
            int i = id.i;
            int j = id.j;
            chunk_map[i][j] = nullptr;
            active_chunks.erase(active_chunks.begin()+(k));
            std::cout << "Unloaded chunk @ CHUNK_ID:" << i << " " << j << std::endl;
        }
    }
}

void Soil::load_chunk(int i, int j) {
    auto soil_chunk = std::make_shared<SoilChunk>();
    soil_chunk->init_chunk(vtx_dims, scale, {i,j,chunk_idx_to_worldpos(i,j) });
    chunk_map[i][j] = soil_chunk;
    active_chunks.push_back(soil_chunk);
    std::cout << "Loading chunk @ CHUNK_ID: " << i << " " << j << " WORLDPOS " << chunk_idx_to_worldpos(i,j) << std::endl;
}

void Soil::unload_chunk(uint32_t i, uint32_t j) {
    chunk_map[i][j] = nullptr;
}

std::shared_ptr<SoilChunk> Soil::get_chunk(uint32_t i, uint32_t j) {
    return chunk_map[i][j];
}
