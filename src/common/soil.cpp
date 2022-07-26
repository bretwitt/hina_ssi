#ifndef SOIL_CPP
#define SOIL_CPP

#include <gazebo/common/common.hh>
#include <utility>
#include "geometry.cpp"

namespace gazebo {

    struct SoilData {
        int x_width = 5;
        int y_width = 5;
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

        Vector3d get_vertex_at_index(uint32_t x, uint32_t y) {
            return (*soil_hashmap)[x][y];
        }

        void set_vertex_at_index(uint32_t x, uint32_t y, const ignition::math::Vector3<double>& vtx) const {
            (*soil_hashmap)[x][y] = vtx;
        }

        void get_nearest_index(Vector2d vtx, uint32_t& x, uint32_t& y) const {
            auto x_unrounded = (vtx.X() / scale) - x_offset;
            auto y_unrounded = (vtx.Y() / scale) - y_offset;

            x = (int)x_unrounded;
            y = (int)y_unrounded;
        }
    };


    class Soil {
    private:
        SoilData* _data;

    public:

        explicit Soil(SoilData* soil_data) {
            this->_data = soil_data;
            if (_data->soil_hashmap == nullptr) {
                _data->init_field();
                generate_soil_vertices();
                generate_indices();
            }
        }

        ~Soil() {
            delete[] _data->soil_hashmap;
            delete[] _data->indices;
            delete _data;
        }

        SoilData* get_data() {
            return _data;
        }

        void generate_soil_vertices() {
            _data->x_offset = -(double) (_data->x_width - 1) / 2;
            _data->y_offset = -(double) (_data->y_width - 1) / 2;

            for (int i = 0; i < _data->x_width; i++) {
                for (int j = 0; j < _data->y_width; j++) {
                    auto i_f = (float) i;
                    auto j_f = (float) j;
                    auto v3 = Vector3d(_data->scale * (i_f + _data->x_offset), _data->scale * (j_f + _data->y_offset), 0.0);

                    _data->set_vertex_at_index(i, j, v3);
                }
            }
        }

        void generate_indices() const {
            uint32_t x_size = _data->x_width;
            uint32_t y_size = _data->y_width;
            auto indices = _data->indices;

            uint32_t idx = 0;
            for (uint32_t y = 0; y < y_size - 1; y++) {
                for (uint32_t x = 0; x < x_size - 1; x++) {
                    uint32_t a = (x_size * x) + y;
                    uint32_t b = (x_size * (x + 1)) + y;
                    uint32_t c = (x_size * (x + 1)) + (y + 1);
                    uint32_t d = (x_size * x) + (y + 1);

                    indices[idx++] = a;
                    indices[idx++] = d;
                    indices[idx++] = c;
                    indices[idx++] = c;
                    indices[idx++] = b;
                    indices[idx++] = a;
                }
            }
        }

        void try_deform(Triangle& meshTri) {
            if (meshTri.as_cgal_tri_proj().is_degenerate()) {
                return;
            }

            std::vector<std::pair<uint32_t, uint32_t>> idx_v;

            get_hash_idx_within_tri_rect_bounds(meshTri, idx_v);

            double w = 0.2f;

            for(const auto& point : idx_v) {
                auto v3 = _data->get_vertex_at_index(point.first, point.second);
                if(penetrates(meshTri, v3, w)) {
                    terramx_deform(point.first, point.second, v3);
                }
            }
        }

        void get_hash_idx_within_tri_rect_bounds(Triangle meshTri, std::vector<std::pair<uint32_t, uint32_t>>& idx) {
            auto max_x = max(meshTri.v1.X(), max(meshTri.v2.X(), meshTri.v3.X()));
            auto max_y = max(meshTri.v1.Y(), max(meshTri.v2.Y(), meshTri.v3.Y()));
            auto min_x = min(meshTri.v1.X(), min(meshTri.v2.X(), meshTri.v3.X()));
            auto min_y = min(meshTri.v1.Y(), min(meshTri.v2.Y(), meshTri.v3.Y()));

            auto scale = _data->scale;

            auto iter_x = ceil( ((max_x - min_x) / scale) );
            auto iter_y = ceil( ((max_y - min_y) / scale) );

            uint32_t x_start = 0;
            uint32_t y_start = 0;

            _data->get_nearest_index(Vector2d(min_x, min_y), x_start, y_start);

            for(uint32_t y = 0; y < iter_y; y++) {
                for(uint32_t x = 0; x < iter_x; x++) {
                    idx.emplace_back(x + x_start,y + y_start);
                }
            }
        }

        bool penetrates(Triangle meshTri, const Vector3d& point, double w) {
            if(intersects_projected(std::move(meshTri), AABB(point, w ))) {
                // Height check
                return true;
            }
            return false;
        }

        bool intersects_projected(Triangle meshTri, AABB vertexRect) {
            return Geometry::intersects_box_tri(std::move(meshTri), std::move(vertexRect)) ;
        }

        void terramx_deform(uint32_t x, uint32_t y, Vector3d v3) {
            auto _v3 = Vector3d(v3.X(), v3.Y(), -1);
            _data->set_vertex_at_index(x, y, _v3);
        }


    };
}

#endif