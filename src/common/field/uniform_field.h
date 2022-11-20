#ifndef HINA_SSI_PLUGIN_UNIFORM_FIELD_H
#define HINA_SSI_PLUGIN_UNIFORM_FIELD_H

#include <cstdint>
#include <unordered_map>
#include <memory>
#include <cmath>
#include <gazebo/common/common.hh>

template<class T>
class FieldVertex {

public:
    Vector3d v3;
    Vector3d v3_0;
    std::shared_ptr<T> v;

    explicit FieldVertex(const Vector3d& v3, const Vector3d& v3_0) {
        this->v3 = v3;
        this->v3_0 = v3_0;
        v = std::make_shared<T>();
    }

    explicit FieldVertex(const Vector3d& v3_0) : FieldVertex(v3_0, v3_0) {}
};

template <class V>
class UniformField {

typedef std::shared_ptr<FieldVertex<V>> FieldVertexVPtr;
typedef std::unordered_map<uint32_t, std::unordered_map<uint32_t, FieldVertexVPtr>> uniform_field;

public:
    std::unique_ptr<uint32_t[]> indices{};
    std::unique_ptr<uniform_field> vertices{};

    uint32_t x_width{};
    uint32_t y_width{};
    double scale{};

    double x_offset = 0;
    double y_offset = 0;

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

    explicit UniformField (uint32_t x_width, uint32_t y_width, double scale) {
        this->x_width = x_width;
        this->y_width = y_width;
        this->scale = scale;
    }

    void init_field() {
        vertices = std::make_unique<uniform_field>();
        indices = std::make_unique<uint32_t[]>((x_width - 1) * (y_width - 1) * 3 * 2);
    }

    FieldVertexVPtr vertex_at_flattened_index(uint32_t idx) {
        uint32_t x;
        uint32_t y;
        unflatten_index(idx, x, y);
        return (*vertices)[x][y];
    }

    void set_vertex_at_flattened_index(uint32_t idx, FieldVertex<V> vtx) {
        uint32_t x;
        uint32_t y;
        unflatten_index(idx, x, y);
        auto _vtx = get_vertex_at_index(x, y);
        *_vtx = vtx;
    }

    void unflatten_index(uint32_t idx, uint32_t& x, uint32_t& y) {
        x = idx % (x_width);
        y = floor(idx / x_width);
    }

    FieldVertexVPtr get_vertex_at_index(uint32_t x, uint32_t y) {
        return (*vertices)[std::min(x, x_width - 1)][std::min(y, y_width - 1)];
    }

    FieldVertexVPtr get_vertex_at_flattened_index(uint32_t idx) {
        uint32_t x;
        uint32_t y;
        unflatten_index(idx, x, y);
        return get_vertex_at_index(x,y);
    }

    void set_vertex_at_index(uint32_t x, uint32_t y, FieldVertexVPtr vtx) {
        (*vertices)[x][y] = std::move(vtx);
    }

    void get_nearest_index(ignition::math::Vector2d vtx, uint32_t& x, uint32_t& y) const {
        x = (int)((vtx.X() / scale) - x_offset);
        y = (int)((vtx.Y() / scale) - y_offset);
    }


};


#endif //HINA_SSI_PLUGIN_UNIFORM_FIELD_H
