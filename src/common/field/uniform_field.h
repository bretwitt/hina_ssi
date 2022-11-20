#ifndef HINA_SSI_PLUGIN_UNIFORM_FIELD_H
#define HINA_SSI_PLUGIN_UNIFORM_FIELD_H

#include <cstdint>
#include <unordered_map>
#include <memory>
#include <cmath>
#include <gazebo/common/common.hh>
#include "field_vertex.h"

namespace hina {
    template<class V>
    class UniformField {

    public:

        typedef std::shared_ptr<FieldVertex<V>> FieldVertexVPtr;
        typedef std::unordered_map<uint32_t, std::unordered_map<uint32_t, FieldVertexVPtr>> UniformFieldMap;

        std::unique_ptr<uint32_t[]> indices{};
        std::unique_ptr<UniformFieldMap> vertices{};

        uint32_t x_width{};
        uint32_t y_width{};
        double scale{};

        double x_offset = 0;
        double y_offset = 0;

        explicit UniformField(uint32_t x_width, uint32_t y_width, double scale) {
            this->x_width = x_width;
            this->y_width = y_width;
            this->scale = scale;

            x_offset = -(double) (x_width - 1) / 2;
            y_offset = -(double) (y_width - 1) / 2;
        }


        void init_field() {
            vertices = std::make_unique<UniformFieldMap>();
            indices = std::make_unique<uint32_t[]>((x_width - 1) * (y_width - 1) * 3 * 2);

            generate_vertices();
            generate_indices();
        }

        FieldVertexVPtr vertex_at_flattened_index(uint32_t idx) {
            uint32_t x;
            uint32_t y;
            unflatten_index(idx, x, y);
            return (*vertices)[x][y];
        }

        void set_vertex_at_flattened_index(uint32_t idx, FieldVertex <V> vtx) {
            uint32_t x;
            uint32_t y;
            unflatten_index(idx, x, y);
            auto _vtx = get_vertex_at_index(x, y);
            *_vtx = vtx;
        }

        FieldVertexVPtr get_vertex_at_index(uint32_t x, uint32_t y) {
            return (*vertices)[std::min(x, x_width - 1)][std::min(y, y_width - 1)];
        }

        FieldVertexVPtr get_vertex_at_flattened_index(uint32_t idx) {
            uint32_t x;
            uint32_t y;
            unflatten_index(idx, x, y);
            return get_vertex_at_index(x, y);
        }

        void unflatten_index(uint32_t idx, uint32_t &x, uint32_t &y) {
            x = idx % (x_width);
            y = floor(idx / x_width);
        }

        void set_vertex_at_index(uint32_t x, uint32_t y, FieldVertexVPtr vtx) {
            (*vertices)[x][y] = std::move(vtx);
        }

        void get_nearest_index(ignition::math::Vector2d vtx, uint32_t &x, uint32_t &y) const {
            x = (int) ((vtx.X() / scale) - x_offset);
            y = (int) ((vtx.Y() / scale) - y_offset);
        }

        void generate_vertices() {
            for (int j = 0; j < y_width; j++) {
                for (int i = 0; i < x_width; i++) {
                    auto i_f = (float) i;
                    auto j_f = (float) j;

                    auto x = scale * (i_f + x_offset);
                    auto y = scale * (j_f + y_offset);

                    auto v3 = Vector3d(x, y, 0);

                    set_vertex_at_index(i, j, std::make_shared<FieldVertex<V>>(v3));
                }
            }
        }

        void generate_indices() const {
            uint32_t x_size = x_width;
            uint32_t y_size = y_width;

            uint32_t idx = 0;
            for (uint32_t x = 0; x < x_size - 1; x++) {
                for (uint32_t y = 0; y < y_size - 1; y++) {
                    uint32_t a = (x_size * y) + x;
                    uint32_t b = (x_size * (y + 1)) + x;
                    uint32_t c = (x_size * (y + 1)) + (x + 1);
                    uint32_t d = (x_size * y) + (x + 1);

                    indices[idx++] = a;
                    indices[idx++] = d;
                    indices[idx++] = c;

                    indices[idx++] = c;
                    indices[idx++] = b;
                    indices[idx++] = a;
                }
            }
        }

    };


}

#endif //HINA_SSI_PLUGIN_UNIFORM_FIELD_H
