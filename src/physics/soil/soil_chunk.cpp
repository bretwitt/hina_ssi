#include "soil_chunk.h"

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
    uint32_t x_start, y_start, x_end, y_end;

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
        if(!v3->v->isAir && penetrates(meshTri, v3, scale)) {
            penetrating_coords.emplace_back(x + x_start, y + y_start, *this, v3);
            terramx_deform(link, meshTri, x + x_start, y + y_start, v3, scale, displaced_vol,
                           p_sampler->get_params_at_index(x + x_start, y + y_start));
        }
    }

    return penetrating_coords;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void SoilChunk::terramx_deform(const physics::LinkPtr &linkPtr, const Triangle &meshTri, uint32_t x, uint32_t y,
                                const std::shared_ptr<FieldVertex<SoilVertex>> &vertex, double w,
                                double &displaced_volume, SoilPhysicsParams vert_attr)
{

    auto v3 = vertex->v3;
    auto v3_0 = vertex->v3_0;

    auto vert_state = vertex->v;

    double k_phi = vert_attr.k_phi;
    double k_e = vert_attr.k_e;

    double y_h = meshTri.centroid().Z();
    double y_r = v3_0.Z();

    double s_y = y_r - y_h;
    double s_p = vert_state->s_p;

    double sigma_t = k_e*(s_y - s_p);
    double sigma_p = 0.0;

    Vector3d normal_dA(0,0,0);

    auto s_sink = 0.0;

    if(sigma_t > 0) {                            // Unilateral Contact
        /* Geometry Calculations */
        auto vtx_ul = this->p_field->get_vertex_at_index(x - 1, y + 1)->v3;
        auto vtx_dl = this->p_field->get_vertex_at_index(x - 1, y - 1)->v3;
        auto vtx_ur = this->p_field->get_vertex_at_index(x + 1, y + 1)->v3;
        auto vtx_dr = this->p_field->get_vertex_at_index(x + 1, y - 1)->v3;

        auto vtx_u = this->p_field->get_vertex_at_index(x, y + 1)->v3;
        auto vtx_d = this->p_field->get_vertex_at_index(x, y - 1)->v3;
        auto vtx_l = this->p_field->get_vertex_at_index(x - 1, y)->v3;
        auto vtx_r = this->p_field->get_vertex_at_index(x + 1, y)->v3;
        const auto &vtx = v3;

        auto tri1 = Triangle(vtx, vtx_ul, vtx_u);
        auto tri2 = Triangle(vtx, vtx_l, vtx_ul);
        auto tri3 = Triangle(vtx, vtx_u, vtx_r);

        auto tri4 = Triangle(vtx, vtx_r, vtx_d);
        auto tri5 = Triangle(vtx, vtx_dr, vtx_d);

        auto tri6 = Triangle(vtx, vtx_d, vtx_l);

        auto normal_sum = (tri1.normal() + tri2.normal() + tri3.normal() + tri4.normal() + tri5.normal() +
                           tri6.normal()).Normalized();

        auto area = w * w;

        normal_dA = -normal_sum * area;

        vert_state->normal = -normal_sum;

        vert_state->normal_dA = normal_dA;

        auto sigma_star = sigma_t;
        s_sink = s_y;

        if(sigma_star < vert_state->sigma_yield) {
            sigma_p = sigma_star;
        } else {
            sigma_p = (k_phi /* + (k_c/B)*/)*(s_y);
            vert_state->sigma_yield = sigma_p;
            auto s_p_o = vert_state->s_p;
            vert_state->s_p = s_sink - (sigma_p / k_e);
            vert_state->s_e = s_sink - vert_state->s_p;
            vert_state->plastic_flow = (vert_state->s_p - s_p_o)*vert_attr.mfr;
        }

        /* update footprints */

        auto v1 = this->p_field->get_vertex_at_index(x, y + 1)->v;
        auto v2 = this->p_field->get_vertex_at_index(x, y - 1)->v;
        auto v3 = this->p_field->get_vertex_at_index(x + 1, y)->v;
        auto v4 = this->p_field->get_vertex_at_index(x - 1, y)->v;

        vertex->v->footprint = 1;
        if(v1->footprint != 1) {
            v1->footprint = 2;
        }
        if(v2->footprint != 1) {
            v2->footprint = 2;
        }
        if(v3->footprint != 1) {
            v3->footprint = 2;
        }
        if(v4->footprint != 1) {
            v4->footprint = 2;
        }
    }

    auto z = v3_0.Z() - s_p;
    auto force_origin = v3_0.Z() - s_sink;

    vert_state->sigma = sigma_p;

    vertex->v3 = Vector3d(v3.X(), v3.Y(), z);

    displaced_volume = vert_state->plastic_flow*w*w;

    auto force_v = Vector3d(
            (vert_attr.c + (vert_state->sigma * tan(vert_attr.phi))) * vert_state->normal_dA.X(),
            (vert_attr.c + (vert_state->sigma * tan(vert_attr.phi))) * vert_state->normal_dA.Y(),
            (vert_state->sigma) * vert_state->normal_dA.Z()
    );

    // Apply f/t
    linkPtr->AddForceAtWorldPosition(force_v, Vector3d(v3_0.X(), v3_0.Y(), force_origin));

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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool SoilChunk::penetrates(const Triangle& meshTri, const std::shared_ptr<FieldVertex<SoilVertex>>& vtx, double w) {
    auto point = vtx->v3;
    return (meshTri.centroid().Z() <= point.Z() && intersects_projected(meshTri, AABB(point, w )));
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool SoilChunk::intersects_projected(const Triangle& meshTri, const AABB& vertexRect) {
    return Geometry::getInstance()->intersects_box_tri(meshTri, vertexRect) ;
};
