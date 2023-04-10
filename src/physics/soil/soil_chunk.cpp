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

void SoilChunk::clear_footprint() {
    auto x_w = this->p_field->x_vert_width;
    auto y_w = this->p_field->y_vert_width;
    for(uint32_t i = 0; i < x_w*y_w; i++) {
        auto v3 = this->p_field->get_vertex_at_flattened_index(i)->v;
        v3->footprint = 0;
    }
}

