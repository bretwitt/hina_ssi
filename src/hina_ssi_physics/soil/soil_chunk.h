#ifndef HINA_SSI_PLUGIN_SOIL_CHUNK_H
#define HINA_SSI_PLUGIN_SOIL_CHUNK_H

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <cmath>

#include "../../common/field/uniform_field.h"
#include "soil_data.h"

using ignition::math::Vector3d;
using namespace hina;

namespace hina {
    class SoilChunk {
    public:
        uint32_t x = 0;
        uint32_t y = 0;
        Vector3d origin;
        double width = 0;
        double height = 0;
        double res = 0;

        std::shared_ptr<UniformField<SoilAttributes>> field = nullptr;

        SandboxConfig config;

        SoilChunk() {
        }


        void init_chunk(FieldTrueDimensions dims, double scale) {
            this->field = std::make_shared<UniformField<SoilAttributes>>(dims, config.scale);
            this->field->init_field();
        }

        void init_chunk(FieldVertexDimensions dims, double scale) {
            this->field = std::make_shared<UniformField<SoilAttributes>>(dims, config.scale);
            this->field->init_field();
        }

        void generate_vertices(SandboxConfig config) {
            for (int j = 0; j < field->y_vert_width; j++) {
                for (int i = 0; i < field->x_vert_width; i++) {
                    auto vertex = field->get_vertex_at_index(i,j);

                    auto x = vertex->v3.X();
                    auto y = vertex->v3.Y();

                    const double z = y*tan(config.angle);

                    auto v3 = Vector3d(x, y, z);

                    field->set_vertex_at_index(i, j, std::make_shared<FieldVertex<SoilAttributes>>(v3));
                }
        }
    };
}


#endif //HINA_SSI_PLUGIN_SOIL_CHUNK_H
