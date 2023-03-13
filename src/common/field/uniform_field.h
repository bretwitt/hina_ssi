#ifndef HINA_SSI_PLUGIN_UNIFORM_FIELD_H
#define HINA_SSI_PLUGIN_UNIFORM_FIELD_H

#include <cstdint>
#include <unordered_map>
#include <memory>
#include <cmath>
#include <gazebo/common/common.hh>

#include "field_vertex.h"
#include "vertex_dims.h"
#include "../../common/field/base_vertex_sampler.h"

using namespace gazebo;

namespace hina {

    struct FieldVector2 {
        double x;
        double y;
    };

    template<class V>
    class UniformField {

    private:
        void load_vertex_dims(uint32_t verts_x, uint32_t verts_y, double scl) {
            this->x_vert_width = verts_x;
            this->y_vert_width = verts_y;
            this->scale = scl;
        }

        void recalculate_width() {
            this->x_width = scale*(x_vert_width - 1);
            this->y_width = scale*(y_vert_width - 1);
        }

    public:
        FieldVector2 origin;

        typedef std::shared_ptr<FieldVertex<V>> FieldVertexVPtr;
        typedef std::unordered_map<uint32_t, std::unordered_map<uint32_t, FieldVertexVPtr>> UniformFieldMap;

        std::unique_ptr<uint32_t[]> indices{};
        std::unique_ptr<UniformFieldMap> vertices{};


        uint32_t x_vert_width{};
        uint32_t y_vert_width{};

        uint32_t x_width{};
        uint32_t y_width{};

        double scale{};

        explicit UniformField(FieldTrueDimensions dims, double resolution) : UniformField(as_vtx_dims(dims,resolution), resolution) {
        }

        explicit UniformField(FieldVertexDimensions dims, double resolution) {
            load_vertex_dims(dims.verts_x, dims.verts_y, resolution);
            recalculate_width();
        }

        explicit UniformField() {
            this->x_vert_width = 5;
            this->y_vert_width = 5;
            this->scale = 0.5;
            recalculate_width();
        }

        static FieldVertexDimensions as_vtx_dims(FieldTrueDimensions true_dims, double resolution) {
            uint32_t verts_x = floor(true_dims.true_x / resolution) + 1;
            uint32_t verts_y = floor(true_dims.true_y / resolution) + 1;
            return FieldVertexDimensions { verts_x, verts_y };
        }

        void init_field(std::shared_ptr<BaseVertexSampler> sampler) {
            vertices = std::make_unique<UniformFieldMap>();
            indices = std::make_unique<uint32_t[]>((x_vert_width - 1) * (y_vert_width - 1) * 3 * 2);

            generate_vertices(sampler);
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

        void set_origin(FieldVector2 origin) {
            this->origin = origin;
        }

        FieldVertexVPtr get_vertex_at_index(uint32_t x, uint32_t y) {
            return (*vertices)[std::min(x, x_vert_width - 1)][std::min(y, y_vert_width - 1)];
        }

        FieldVertexVPtr get_vertex_at_flattened_index(uint32_t idx) {
            uint32_t x;
            uint32_t y;
            unflatten_index(idx, x, y);
            return get_vertex_at_index(x, y);
        }

        void unflatten_index(uint32_t idx, uint32_t &x, uint32_t &y) {
            x = idx % (x_vert_width);
            y = floor(idx / x_vert_width);
        }

        void set_vertex_at_index(uint32_t x, uint32_t y, FieldVertexVPtr vtx) {
            (*vertices)[x][y] = std::move(vtx);
        }

        void get_nearest_index(ignition::math::Vector2d vtx, uint32_t &x, uint32_t &y) const {
            x = (int) (floor(vtx.X() / scale));
            y = (int) (floor(vtx.Y() / scale));
        }

        void generate_vertices(std::shared_ptr<BaseVertexSampler> sampler) {
            for (int j = 0; j < y_vert_width; j++) {
                for (int i = 0; i < x_vert_width; i++) {
                    auto i_f = (float) i;
                    auto j_f = (float) j;

                    auto x = (scale * i_f) + origin.x;
                    auto y = (scale * j_f) + origin.y;
                    double z = sampler->get_z_at_index(x,y);

                    auto v3 = Vector3d(x, y, z);

                    set_vertex_at_index(i, j, std::make_shared<FieldVertex<V>>(v3));
                }
            }
        }

        void generate_indices() const {
            uint32_t x_size = x_vert_width;
            uint32_t y_size = y_vert_width;

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
