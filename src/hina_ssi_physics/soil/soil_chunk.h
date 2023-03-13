#ifndef HINA_SSI_PLUGIN_SOIL_CHUNK_H
#define HINA_SSI_PLUGIN_SOIL_CHUNK_H

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <cmath>
#include "../../common/geometry.h"
#include "../../common/field/uniform_field.h"
#include "soil_vertex.h"
#include "soil_chunk_location.h"
#include "../../common/field/base_vertex_sampler.h"
#include "../soil/soil_vertex_sampler.h"
#include <ignition/common/Profiler.hh>

using ignition::math::Vector3d;
using ignition::math::Vector2d;

namespace hina {


    class SoilChunk {

    public:
        SandboxConfig config;
        SoilChunkLocationMetadata location;

        Vector2d max;
        Vector2d min;

        double width = 0;
        double height = 0;
        double res = 0;

        std::shared_ptr<SoilVertexSampler> sampler = nullptr;

        std::shared_ptr <UniformField<SoilVertex>> field = nullptr;

        SoilChunk() = default;

        void init_chunk(FieldVertexDimensions dims, double scale, SoilChunkLocationMetadata location, const std::shared_ptr<SoilVertexSampler>& sampler) {
            this->field = std::make_shared<UniformField<SoilVertex>>(dims, scale);
            this->field->set_origin({location.origin.X(), location.origin.Y()});
            this->field->init_field(sampler);
            this->sampler = sampler;
            this->location = location;
        }

        typedef std::vector<std::tuple<uint32_t, uint32_t, SoilChunk, std::shared_ptr<FieldVertex<SoilVertex>>>> FieldV;
        FieldV try_deform(const Triangle& meshTri, const physics::LinkPtr& link, double& displaced_vol, double dt) {

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
            scale = field->scale;

            // Get first soil vertex coordinates (x_start, y_start)
            x_start = 0;
            y_start = 0;
            field->get_nearest_index(Vector2d(min_x, min_y), x_start, y_start);

            // Get last soil vertex coordinates (x_end, y_end)
//            x_end = 0;
//            y_end = 0;
//            field->get_nearest_index(Vector2d(max_x, max_y), x_end, y_end);
//
//            // Calculate number of iterations required in each direction
//            iter_x = x_end - x_start + 1;
//            iter_y = y_end - y_start + 1;

            iter_x = ceil( ((max_x - min_x) / scale) ) + 1;
            iter_y = ceil( ((max_y - min_y) / scale) ) + 1;

            // Search soil field within bounds under AABB
            FieldV penetrating_coords;

            for(uint32_t k = 0; k < iter_x*iter_y; k++) {
                uint32_t y = floor(k / iter_y);                                 // Unpack for loop into (x,y) index
                uint32_t x = k - (iter_x*y);
                auto v3 = field->get_vertex_at_index(x + x_start, y + y_start);

                // If mesh tri penetrates vtx
                if(!v3->v->isAir && penetrates(meshTri, v3, scale)) {
                    penetrating_coords.emplace_back(x + x_start, y + y_start, *this, v3);
                    terramx_deform(link, meshTri, x + x_start, y + y_start, v3, scale, dt, displaced_vol);
                }
                //penetrating_coords.emplace_back(x + x_start, y + y_start, *this, v3);
            }

            return penetrating_coords;
        };


        void terramx_deform(const physics::LinkPtr &linkPtr, const Triangle &meshTri, uint32_t x, uint32_t y,
                            const std::shared_ptr<FieldVertex<SoilVertex>> &vertex, double w, double dt,
                            double &displaced_volume)
        {

            auto v3 = vertex->v3;
            auto v3_0 = vertex->v3_0;

            auto vert_attr = this->sampler->get_params_at_index(x,y);

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
                auto vtx_ul = field->get_vertex_at_index(x - 1, y + 1)->v3;
                auto vtx_dl = field->get_vertex_at_index(x - 1, y - 1)->v3;
                auto vtx_ur = field->get_vertex_at_index(x + 1, y + 1)->v3;
                auto vtx_dr = field->get_vertex_at_index(x + 1, y - 1)->v3;

                auto vtx_u = field->get_vertex_at_index(x, y + 1)->v3;
                auto vtx_d = field->get_vertex_at_index(x, y - 1)->v3;
                auto vtx_l = field->get_vertex_at_index(x - 1, y)->v3;
                auto vtx_r = field->get_vertex_at_index(x + 1, y)->v3;
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

                auto v1 = field->get_vertex_at_index(x, y + 1)->v;
                auto v2 = field->get_vertex_at_index(x, y - 1)->v;
                auto v3 = field->get_vertex_at_index(x + 1, y)->v;
                auto v4 = field->get_vertex_at_index(x + 1, y)->v;

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
            //std::cout << displaced_volume << std::endl;

            auto force_v = Vector3d(
                    (vert_attr.c + (vert_state->sigma * tan(vert_attr.phi))) * vert_state->normal_dA.X(),
                    (vert_attr.c + (vert_state->sigma * tan(vert_attr.phi))) * vert_state->normal_dA.Y(),
                    (vert_state->sigma) * vert_state->normal_dA.Z()
            );

            // Apply f/t
            linkPtr->AddForceAtWorldPosition(force_v, Vector3d(v3_0.X(), v3_0.Y(), force_origin));

        };

        void clear_footprint() {
            auto x_w = field->x_vert_width;
            auto y_w = field->y_vert_width;
            for(uint32_t i = 0; i < x_w*y_w; i++) {
                auto v3 = field->get_vertex_at_flattened_index(i)->v;
                v3->footprint = 0;
            }
        }

        static bool intersects_projected(const Triangle& meshTri, const AABB& vertexRect) {
            return Geometry::getInstance()->intersects_box_tri(meshTri, vertexRect) ;
        };

        static bool penetrates(const Triangle& meshTri, const std::shared_ptr<FieldVertex<SoilVertex>>& vtx, double w) {
            auto point = vtx->v3;
            return (meshTri.centroid().Z() <= point.Z() && intersects_projected(meshTri, AABB(point, w )));
        };

    };
}


#endif //HINA_SSI_PLUGIN_SOIL_CHUNK_H
