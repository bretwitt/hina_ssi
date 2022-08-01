#ifndef HINA_SSI_PLUGIN_SOIL_DATA_H
#define HINA_SSI_PLUGIN_SOIL_DATA_H

#include <gazebo/common/common.hh>

using ignition::math::Vector3d;
using ignition::math::Vector2d;

namespace gazebo {
    struct SoilData {
        uint32_t x_width = 5;
        uint32_t y_width = 5;
        double scale = 1.0;
        double x_offset = 0;
        double y_offset = 0;
        uint32_t *indices{};
        std::unordered_map<uint32_t, std::unordered_map<uint32_t, Vector3d>>* soil_hashmap{};

        SoilData(int x_width, int y_width, double scale) {
            this->x_width = x_width;
            this->y_width = y_width;
            this->scale = scale;
        }

        ~SoilData() = default;


        void init_field() {
            soil_hashmap = new std::unordered_map<uint32_t, std::unordered_map<uint32_t, Vector3d>>();
            indices = new uint32_t[(x_width - 1) * (y_width - 1) * 3 * 2];
        }

        Vector3d vertex_at_flattened_index(uint32_t idx) const {
            uint32_t x;
            uint32_t y;
            unflatten_index(idx, x, y);
            return (*soil_hashmap)[x][y];
        }

        void set_vertex_at_flattened_index(uint32_t idx, Vector3d vtx) const {
            uint32_t x;
            uint32_t y;
            unflatten_index(idx, x, y);
            (*soil_hashmap)[x][y] = vtx;
        }

        void unflatten_index(uint32_t idx, uint32_t& x, uint32_t& y) const {
            x = idx % (x_width);
            y = floor(idx / x_width);
        }

        Vector3d get_vertex_at_index(uint32_t x, uint32_t y) const {
            return (*soil_hashmap)[std::min(x, x_width - 1)][std::min(y, y_width - 1)];
        }

        void set_vertex_at_index(uint32_t x, uint32_t y, const ignition::math::Vector3<double>& vtx) const {
            (*soil_hashmap)[x][y] = vtx;
        }

        void get_nearest_index(Vector2d vtx, uint32_t& x, uint32_t& y) const {
            x = (int)((vtx.X() / scale) - x_offset);
            y = (int)((vtx.Y() / scale) - y_offset);
        }
    };
}
#endif //HINA_SSI_PLUGIN_SOIL_DATA_H
