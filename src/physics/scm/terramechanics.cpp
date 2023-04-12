#include "terramechanics.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void hina::Terramechanics::terramx_deform(const Triangle &meshTri,
                                          const std::shared_ptr<FieldVertex<SoilVertex>> &vertex, double w,
                                          double &displaced_volume, SoilPhysicsParams vert_attr,
                                          VtxNeighbors normals,
                                          Vector3d& force_v, Vector3d& force_origin)
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
        auto vtx_ul = normals.vtx_ul->v3;
        auto vtx_dl = normals.vtx_dl->v3;
        auto vtx_ur = normals.vtx_ur->v3;
        auto vtx_dr = normals.vtx_dr->v3;

        auto vtx_u = normals.vtx_u->v3;
        auto vtx_d = normals.vtx_d->v3;
        auto vtx_l = normals.vtx_l->v3;
        auto vtx_r = normals.vtx_r->v3;

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

        auto v1 = normals.vtx_ul->v;
        auto v2 = normals.vtx_d->v;
        auto v3 = normals.vtx_l->v;
        auto v4 = normals.vtx_r->v;

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

    force_origin = Vector3d(v3_0.X(), v3_0.Y(), v3_0.Z() - s_sink);

    vert_state->sigma = sigma_p;

    vertex->v3 = Vector3d(v3.X(), v3.Y(), z);

    displaced_volume = vert_state->plastic_flow*w*w;

    force_v = Vector3d(
            (vert_attr.c + (vert_state->sigma * tan(vert_attr.phi))) * vert_state->normal_dA.X(),
            (vert_attr.c + (vert_state->sigma * tan(vert_attr.phi))) * vert_state->normal_dA.Y(),
            (vert_state->sigma) * vert_state->normal_dA.Z()
    );


}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
