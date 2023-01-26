#include "soil.h"

using namespace gazebo;
using namespace hina;

Soil::Soil(FieldVertexDimensions dims, double scale) {
    this->sc.init_chunk(dims, scale, {0,0, Vector2d(0,0)});
}

Soil::Soil(FieldTrueDimensions dims, double scale) {
    this->sc.init_chunk(dims, scale, {0,0, Vector2d(0,0)});
}

Soil::Soil(SandboxConfig config) : Soil(FieldTrueDimensions { config.x_width, config.y_width }, config.scale) {
}

Soil::Soil(const std::shared_ptr<DEM>& dem) : Soil(FieldVertexDimensions { dem->field->y_vert_width, dem->field->x_vert_width }, dem->field->scale) {
    load_dem_geometry(dem);
}

void Soil::generate_sandbox_geometry(SandboxConfig config) {
    this->sc.generate_vertices(config);
}

void Soil::load_dem_geometry(const std::shared_ptr<DEM>& dem) const {
    auto field = sc.field;
    for(int i = 0; i < dem->field->y_vert_width; i++) {
        for(int j = 0; j < dem->field->x_vert_width; j++) {
            Vector3d dem_v3 = dem->field->get_vertex_at_index(i,j)->v3;
            Vector3d v3 ( dem_v3.X(), dem_v3.Y(), dem_v3.Z());
            field->set_vertex_at_index(i, j, std::make_shared<FieldVertex<SoilAttributes>>(v3));
        }
    }
}

std::vector<std::tuple<uint32_t, uint32_t, std::shared_ptr<FieldVertex<SoilAttributes>>>> Soil::try_deform(const Triangle& meshTri, const physics::LinkPtr& link, float dt, float& displaced_volume) {
    return sc.try_deform(meshTri, link, dt);
}

SoilChunk Soil::get_chunk() {
    return sc;
}

