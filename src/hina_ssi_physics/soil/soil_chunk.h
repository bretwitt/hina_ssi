#ifndef HINA_SSI_PLUGIN_SOIL_CHUNK_H
#define HINA_SSI_PLUGIN_SOIL_CHUNK_H

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <cmath>
#include "../../common/geometry.h"
#include "../../common/field/uniform_field.h"
#include "soil_data.h"

using ignition::math::Vector3d;
using ignition::math::Vector2d;

namespace hina {

    struct SoilChunkLocationMetadata {
        int i;
        int j;
        Vector2d origin; // Vector2d of first vertex
    };

    class SoilChunk {
    public:

        SoilChunkLocationMetadata location;

        double width = 0;
        double height = 0;
        double res = 0;


        std::shared_ptr <UniformField<SoilAttributes>> field = nullptr;

        SandboxConfig config;

        SoilChunk() {
        }

        void init_chunk(FieldTrueDimensions dims, double scale, SoilChunkLocationMetadata location) {
            this->field = std::make_shared<UniformField<SoilAttributes>>(dims, scale);
            this->field->set_origin({location.origin.X(), location.origin.Y()});
            this->field->init_field();
            this->location = location;
        }

        void init_chunk(FieldVertexDimensions dims, double scale, SoilChunkLocationMetadata location) {
            this->field = std::make_shared<UniformField<SoilAttributes>>(dims, scale);
            this->field->set_origin({location.origin.X(), location.origin.Y()});
            this->field->init_field();
            this->location = location;
        }

        void generate_vertices(SandboxConfig config) {
            for (int j = 0; j < field->y_vert_width; j++) {
                for (int i = 0; i < field->x_vert_width; i++) {
                    auto vertex = field->get_vertex_at_index(i, j);

                    auto x = vertex->v3.X();
                    auto y = vertex->v3.Y();

                    const double z = y * tan(config.angle);

                    auto v3 = Vector3d(x, y, z);

                    field->set_vertex_at_index(i, j, std::make_shared<FieldVertex<SoilAttributes>>(v3));
                }
            }
        }

        typedef std::vector<std::tuple<uint32_t, uint32_t, std::shared_ptr<FieldVertex<SoilAttributes>>>> FieldV;
        FieldV try_deform(const Triangle& meshTri, const physics::LinkPtr& link, float dt) {
            double max_x, max_y, min_x, min_y;
            double scale;
            int iter_x, iter_y;
            uint32_t x_start, y_start;

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

            // Calculate number of iterations required in each direction
            iter_x = ceil( ((max_x - min_x) / scale) ) + 1;
            iter_y = ceil( ((max_y - min_y) / scale) ) + 1;

            // Search soil field within bounds under AABB
            std::vector<std::tuple<uint32_t, uint32_t, std::shared_ptr<FieldVertex<SoilAttributes>>>> penetrating_coords;

            for(uint32_t k = 0; k < iter_x*iter_y; k++) {
                uint32_t y = floor(k / iter_y);             // Unpack for loop into (x,y) index
                uint32_t x = k - (iter_x*y);
                std::shared_ptr<FieldVertex<SoilAttributes>> v3 = field->get_vertex_at_index(x + x_start, y + y_start);
                if(!v3->v->isAir && penetrates(meshTri, v3, scale)) {
                    penetrating_coords.emplace_back(x + x_start,y + y_start,v3);
                }
            }

            // For each penerated vertex, perform deformation
            // TODO: CUDA this
            for(auto & penetrating_coord : penetrating_coords) {
                auto x = std::get<0>(penetrating_coord);
                auto y = std::get<1>(penetrating_coord);
                auto v3 = std::get<2>(penetrating_coord);

                float displaced_volume_vtx = 0.0f;

                terramx_deform(link, meshTri, x, y, v3, scale, dt, displaced_volume_vtx);
            }
            return penetrating_coords;
        };


        void terramx_deform(const physics::LinkPtr &linkPtr, const Triangle &meshTri, uint32_t x, uint32_t y,
                            const std::shared_ptr<FieldVertex<SoilAttributes>> &vertex, double w, float dt,
                            float &displaced_volume)
        {

            auto v3 = vertex->v3;
            auto v3_0 = vertex->v3_0;

            auto vert_attr = vertex->v;

            double k_phi = vert_attr->k_phi;
            double k_e = vert_attr->k_e;

            double y_h = meshTri.centroid().Z();
            double y_r = v3_0.Z();

            double s_y = y_r - y_h;
            double s_p = vert_attr->s_p;

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
                //auto area = (tri1.area() + tri2.area() + tri3.area() + tri4.area() + tri5.area() + tri6.area()) / 3;
                auto area = w * w;

                normal_dA = -normal_sum * area;

                vert_attr->normal_dA = normal_dA;


                auto sigma_star = sigma_t;
                s_sink = s_y;

                if(sigma_star < vert_attr->sigma_yield) {
                    sigma_p = sigma_star;
                } else {
                    sigma_p = (k_phi /* + (k_c/B)*/)*(s_y);
                    vert_attr->sigma_yield = sigma_p;
                    auto s_p_o = s_p;
                    vert_attr->s_p = s_sink - (sigma_p / k_e);
                    vert_attr->s_e = s_sink - s_p;
                    vert_attr->plastic_flow = -(s_p - s_p_o)*dt;
                }
            }

            auto z = v3_0.Z() - s_p;
            auto force_origin = v3_0.Z() - s_sink;

            vert_attr->sigma = sigma_p;

            vertex->v3 = Vector3d(v3.X(), v3.Y(), z);


            auto force_v = Vector3d(
                    (vert_attr->c + (vert_attr->sigma * tan(vert_attr->phi))) * vert_attr->normal_dA.X(),
                    (vert_attr->c + (vert_attr->sigma * tan(vert_attr->phi))) * vert_attr->normal_dA.Y(),
                    (vert_attr->sigma /*- (plastic_flow)*/) * vert_attr->normal_dA.Z()
            );

            // Apply f/t
            linkPtr->AddForceAtWorldPosition(force_v, Vector3d(v3_0.X(), v3_0.Y(), force_origin));
        };

        bool intersects_projected(const Triangle& meshTri, const AABB& vertexRect) {
            return Geometry::getInstance()->intersects_box_tri(meshTri, vertexRect) ;
        };

        bool penetrates(const Triangle& meshTri, const std::shared_ptr<FieldVertex<SoilAttributes>>& vtx, double w) {
            auto point = vtx->v3;
            return (meshTri.centroid().Z() <= point.Z() && intersects_projected(meshTri, AABB(point, w )));
        };
//
//        void generate_vertices(DEM dem) {
//
//        }
    };
}


#endif //HINA_SSI_PLUGIN_SOIL_CHUNK_H
