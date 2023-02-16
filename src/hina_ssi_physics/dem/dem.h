#ifndef HINA_SSI_PLUGIN_DEM
#define HINA_SSI_PLUGIN_DEM

#include "../soil/soil.h"
#include <cmath>
#include <omp.h>
#include <libInterpolate/Interpolate.hpp>

using namespace gazebo;

namespace hina {
    class DEM {

    public:
        std::shared_ptr<UniformField<SoilAttributes>> field = nullptr;

        DEM(FieldVertexDimensions dims, double scale) {
            field = std::make_shared<UniformField<SoilAttributes>>(dims, scale);
            //field->init_field();
        }

        DEM(FieldTrueDimensions dims, double scale) {
            field = std::make_shared<UniformField<SoilAttributes>>(dims, scale);
            //field->init_field();
        }

        void load_vertex(uint32_t flattened_index, double z) {
            auto vert = field->get_vertex_at_flattened_index(flattened_index);
            Vector3d v3(vert->v3.X(), vert->v3.Y(), z);
            vert->v3 = v3;
            vert->v3_0 = v3;
        }

        void upsample(double new_res) {

            if(new_res >= field->scale) {
                return;
            }

            double x_width = field->x_width;
            double y_width = field->y_width;

            uint32_t verts_x = field->x_vert_width;
            uint32_t verts_y = field->y_vert_width;

            auto new_field = std::make_shared<UniformField<SoilAttributes>>(FieldTrueDimensions { static_cast<double>(x_width), static_cast<double>(y_width) }, new_res);
            //new_field->init_field();

            uint32_t verts_x_p = new_field->x_vert_width;
            uint32_t verts_y_p = new_field->y_vert_width;

            double factor_x = ceil((double)verts_x_p / verts_x);
            double factor_y = ceil((double)verts_y_p / verts_y);

            std::vector<double> field_x;
            std::vector<double> field_y;
            std::vector<double> field_z;

            for(uint32_t x = 0; x < verts_x + 1; x++) {
                for(uint32_t y = 0; y < verts_y + 1; y++) {
                    auto vertex = field->get_vertex_at_index(x,y);
                    double new_x = factor_x * ((double)x);
                    double new_y = factor_y * ((double)y);

                    field_x.push_back(new_x);
                    field_y.push_back(new_y);
                    field_z.push_back(vertex->v3.Z());

                }
            }
            _2D::BicubicInterpolator<double> bicubicInterp;
            bicubicInterp.setData(field_x, field_y, field_z);

            for(uint32_t y = 0; y < verts_y_p + 1; y++) {
                for(uint32_t x = 0; x < verts_x_p + 1; x++) {
                    auto vert = new_field->get_vertex_at_index(x,y);
                    double z = bicubicInterp(x,y);
                    auto v3 = vert->v3;
                    vert->v3 = Vector3d(v3.X(), v3.Y(), z);
                    vert->v3_0 = Vector3d(v3.X(), v3.Y(), z);
                }
            }

            field = new_field;
        }
    };
}

#endif

