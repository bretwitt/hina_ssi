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

hina::Footprint SoilChunk::try_deform(const TriangleContext& triCtx, const physics::LinkPtr& link) {
    double max_x, max_y, min_x, min_y;
    double scale;
    uint32_t iter_x, iter_y;
    uint32_t x_start, y_start, x_end, y_end;

    double chunk_x = location.origin.X();
    double chunk_y = location.origin.Y();

    auto meshTri = triCtx.tri;

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
    Footprint footprint {};
//    int i = 0;

    for(uint32_t k = 0; k < iter_x*iter_y; k++) {

        uint32_t y = floor( k / iter_y);
        uint32_t x = k % iter_y;

        auto v3 = this->p_field->get_vertex_at_index(x + x_start, y + y_start);
        Vector3d force_x{};
        Vector3d force_z{};

        // If mesh tri penetrates vtx
        if(!v3->v->isAir && penetrates(meshTri, v3, scale)) {
            double sinkage = 1;
            terramx_deform(link, triCtx, x + x_start, y + y_start, v3, scale,
                           p_sampler->get_params_at_index(x + x_start, y + y_start), force_z, force_x,
                           sinkage);

            hina::Contact contact { x+x_start,y+y_start, chunk_x, chunk_y, v3 };
            footprint.footprint.push_back(contact);
            footprint.force_x += force_x;
            footprint.force_z += force_z;

            if(sinkage < footprint.max_sinkage) {
                footprint.max_sinkage = sinkage;
            }
        }
    }
    return footprint;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void SoilChunk::terramx_deform(const physics::LinkPtr &linkPtr, const TriangleContext &tri_ctx, uint32_t x, uint32_t y,
                                const std::shared_ptr<FieldVertex<SoilVertex>> &vertex, double w,
                                SoilPhysicsParams vert_attr, Vector3d& normal_force, Vector3d& traction_force,
                                double& sinkage)
{

    Vector3d normal_dA(0,0,0);

    auto& meshTri = tri_ctx.tri;

    if(vertex->v3.Z() > meshTri.centroid().Z()) {                            // Unilateral Contact
        /* Geometry Calculations */
        auto vtx_ul = this->p_field->get_vertex_at_index(x - 1, y + 1)->v3;
        auto vtx_dl = this->p_field->get_vertex_at_index(x - 1, y - 1)->v3;
        auto vtx_ur = this->p_field->get_vertex_at_index(x + 1, y + 1)->v3;
        auto vtx_dr = this->p_field->get_vertex_at_index(x + 1, y - 1)->v3;

        auto vtx_u = this->p_field->get_vertex_at_index(x, y + 1)->v3;
        auto vtx_d = this->p_field->get_vertex_at_index(x, y - 1)->v3;
        auto vtx_l = this->p_field->get_vertex_at_index(x - 1, y)->v3;
        auto vtx_r = this->p_field->get_vertex_at_index(x + 1, y)->v3;
        const auto &vtx = vertex->v3;
        // Construct triangles around the vertex
        auto tri1 = Triangle(vtx, vtx_ul, vtx_u);
        auto tri2 = Triangle(vtx, vtx_l, vtx_ul);
        auto tri3 = Triangle(vtx, vtx_u, vtx_r);
        auto tri4 = Triangle(vtx, vtx_r, vtx_d);
        auto tri5 = Triangle(vtx, vtx_dr, vtx_d);
        auto tri6 = Triangle(vtx, vtx_d, vtx_l);

        // Find the average normal
        auto normal_sum = (tri1.normal() + tri2.normal() + tri3.normal() + tri4.normal() + tri5.normal() +
                           tri6.normal()).Normalized();

        // Area of AABB
//        auto area = w * w;
//        auto area = (tri1.area() + tri2.area() + tri3.area() + tri4.area() + tri5.area() + tri6.area())*0.5;
        auto area = w*w;

        // Calculate attributes of the vertex
        normal_dA = -normal_sum * area;
        vertex->v->normal = -normal_sum;
        vertex->v->normal_dA = normal_dA;

        // Compute contact force and sinkage
        Vector3d normal{};
        Vector3d traction{};
        sinkage = 0;
        this->terramx_contact(vert_attr,tri_ctx, vertex, area, normal, traction, sinkage);

        // Update sinkage of vertex
        const auto v3 = vertex->v3;
        const auto v3_0 = vertex->v3_0;
        vertex->v3 = Vector3d(v3_0.X(), v3_0.Y(), sinkage);

        // Apply f/t

        normal_force = normal; // What's visualized
        traction_force = traction;

        /* update footprints */

        auto _v1 = this->p_field->get_vertex_at_index(x, y + 1)->v;
        auto _v2 = this->p_field->get_vertex_at_index(x, y - 1)->v;
        auto _v3 = this->p_field->get_vertex_at_index(x + 1, y)->v;
        auto _v4 = this->p_field->get_vertex_at_index(x - 1, y)->v;

        vertex->v->footprint = 1;
        if(_v1->footprint != 1) {
            _v1->footprint = 2;
        }
        if(_v2->footprint != 1) {
            _v2->footprint = 2;
        }
        if(_v3->footprint != 1) {
            _v3->footprint = 2;
        }
        if(_v4->footprint != 1) {
            _v4->footprint = 2;
        }
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Input: Contact triangle, contact point, contact point normal, contact aabb area
// Output: Force on contact triangle
void SoilChunk::terramx_contact(const SoilPhysicsParams& vert_attr,
                                const TriangleContext& tri_ctx,
                                const std::shared_ptr<FieldVertex<SoilVertex>>& soil_vertex,
                                const double& aabb_point_area,
                                Vector3d& normal_force,
                                Vector3d& traction_force,
                                double& sinkage) {

    auto contact_tri = tri_ctx.tri;
    double penetration = soil_vertex->v3_0.Z() - contact_tri.centroid().Z();

/////// Tasora

    auto vert_state = soil_vertex->v;
    auto v3_0 = soil_vertex->v3_0;
    auto k_e = 4e7;
    auto s_sink = 0.0;

    double y_h = tri_ctx.tri.centroid().Z();
    double y_r = v3_0.Z();
    double s_y = y_r - y_h;
    double s_p = vert_state->s_p;
    double sigma_t = k_e*(s_y - s_p);

    auto sigma_star = sigma_t;
    auto sigma_p = 0.0;
    s_sink = s_y;

    if(sigma_star < vert_state->sigma_yield) {
        sigma_p = sigma_star;
    } else {
        auto B = tri_ctx.B;
        if(B == 0) {
            B = 9999;
        }

        sigma_p = (814000 + (1.37e3/0.3))*(s_y);
        vert_state->sigma_yield = sigma_p;
        auto s_p_o = vert_state->s_p;
        vert_state->s_p = s_sink - (sigma_p / k_e);
        vert_state->s_e = s_sink - vert_state->s_p;
    }

    sinkage = v3_0.Z() - vert_state->s_p;
    double sigma = sigma_p;
    double force_z = sigma*aabb_point_area;
    Vector3d contact_normal = contact_tri.normal().Normalize();

    // Transform into inertial frame
    // Form bases
    normal_force = -contact_normal*force_z*Vector3d(1,1,1);

    soil_vertex->v->s_sink = s_sink;

////// Shear forces
    double j = tri_ctx.shear_displacement;
    auto slip_vel = tri_ctx.slip_velocity;

    double tau_max = 3500 + (sigma*tan(0.524));
    double tau = tau_max*(1-exp(-j/0.01));

    double force_x = tau*aabb_point_area;

    auto contact_tangent = slip_vel.Normalize();

    // Transform into inertial frame
    // Form bases
    traction_force = slip_vel.Normalize()*force_x;
////// Lateral forces

////// Apply forces


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
