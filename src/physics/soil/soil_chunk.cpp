#include "soil_chunk.h"
#include "../scm/terramechanics.h"

using namespace hina;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void SoilChunk::init_chunk(FieldVertexDimensions dims,
                           double scale, SoilChunkLocation location,
                           const std::shared_ptr<SoilVertexSampler>& sampler) {
    this->p_field = std::make_shared<UniformField<SoilVertex>>(dims, scale);
    this->p_field->set_origin({location.origin.X(), location.origin.Y()});
    this->p_field->init_field(sampler);
    this->p_sampler = sampler;
    this->location = location;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

hina::SoilChunk::Footprint_V SoilChunk::try_deform(const Triangle& meshTri, const physics::LinkPtr& link,
                                                   double& displaced_vol) {

    double max_x, max_y, min_x, min_y;
    double scale;
    uint32_t iter_x, iter_y;
    uint32_t x_start, y_start;

    double chunk_x = location.origin.X();
    double chunk_y = location.origin.Y();

    // AABB bounds under mesh triangle
    max_x = fmax(meshTri.v1.X(), fmax(meshTri.v2.X(), meshTri.v3.X())) - chunk_x;
    max_y = fmax(meshTri.v1.Y(), fmax(meshTri.v2.Y(), meshTri.v3.Y())) - chunk_y;
    min_x = fmin(meshTri.v1.X(), fmin(meshTri.v2.X(), meshTri.v3.X())) - chunk_x;
    min_y = fmin(meshTri.v1.Y(), fmin(meshTri.v2.Y(), meshTri.v3.Y())) - chunk_y;

    // Start constructing bounds to search soil field
    scale = this->p_field->scale;

    // Get first soil vertex coordinates (x_start, y_start)
    x_start = 0;
    y_start = 0;
    this->p_field->get_nearest_index(Vector2d(min_x, min_y), x_start, y_start);

    min_x = min_x - fmod(min_x, scale);
    max_x = (max_x+scale) - fmod(max_x, scale);

    min_y = min_y - fmod(min_y, scale);
    max_y = (max_y+scale) - fmod(max_y, scale);

    iter_x = ceil( ((max_x - min_x) / scale) );
    iter_y = ceil( ((max_y - min_y) / scale) );


    // Search soil field within bounds under AABB
    Footprint_V penetrating_coords;

    for(uint32_t k = 0; k < iter_x*iter_y; k++) {

        uint32_t y = floor( k / iter_y);
        uint32_t x = k % iter_y;

        auto v3 = this->p_field->get_vertex_at_index(x + x_start, y + y_start);

        // If mesh tri penetrates vtx
        if(!v3->v->isAir && Terramechanics::penetrates(meshTri, v3, scale)) {
            Vector3d force_v,v3_0,force_origin;
            penetrating_coords.emplace_back(x + x_start, y + y_start, *this, v3);
            Terramechanics::terramx_deform(meshTri, x + x_start, y + y_start, v3, scale, displaced_vol,
                           p_sampler->get_params_at_index(x + x_start, y + y_start), this->p_field, force_v,force_origin);
            // Apply f/t
            link->AddForceAtWorldPosition(force_v, force_origin);
        }
    }

    return penetrating_coords;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void SoilChunk::clear_footprint() {
    auto x_w = this->p_field->x_vert_width;
    auto y_w = this->p_field->y_vert_width;
    for(uint32_t i = 0; i < x_w*y_w; i++) {
        auto v3 = this->p_field->get_vertex_at_flattened_index(i)->v;
        v3->footprint = 0;
    }
}

