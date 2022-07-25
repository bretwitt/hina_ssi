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

        ~SoilData() {
        }


        void init_field() {
            soil_hashmap = new std::unordered_map<uint32_t, std::unordered_map<uint32_t, Vector3d>>();
            indices = new uint32_t[(x_width - 1) * (y_width - 1) * 3 * 2];
        }

        void reset_collisions() {

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

        void get_nearest_index(Vector3d vtx, uint32_t& x, uint32_t& y) const {
            auto x_unrounded = (vtx.X() - x_offset) / scale;
            auto y_unrounded = (vtx.Y() - x_offset) / scale;
            x = (int)x_unrounded;
            y = (int)y_unrounded;
        }
    };


    class Soil {
    private:
        SoilData _data;

    public:

        explicit Soil(SoilData soil_data) {
            this->_data = soil_data;
            if (_data.soil_hashmap == nullptr) {
                _data.init_field();
                generate_soil_vertices();
                generate_indices();
            }
        }

        ~Soil() {
            delete[] _data.soil_hashmap;
            delete[] _data.indices;
        }

        SoilData get_data() {
            return _data;
        }

        void generate_soil_vertices() {
            _data.x_offset = -(double) _data.x_width / 2;
            _data.y_offset = -(double) _data.y_width / 2;

            for (int i = 0; i < _data.x_width; i++) {
                for (int j = 0; j < _data.y_width; j++) {
                    auto i_f = (float) i;
                    auto j_f = (float) j;
                    auto v3 = Vector3d(_data.scale * (i_f + _data.x_offset), _data.scale * (j_f + _data.y_offset), 0.0);
                    _data.set_vertex_at_index(i, j, v3);
                }
            }
        }

        void generate_indices() const {
            uint32_t x_size = _data.x_width;
            uint32_t y_size = _data.y_width;
            auto indices = _data.indices;

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

        void find_intersection_candidates(const Triangle& meshTri, std::vector<std::pair<uint32_t, uint32_t>>& candidates) {
            _data.reset_collisions();

            std::vector<Vector3d> points;
            generate_sampling_points(meshTri,points);

            for(auto& point : points) {
                uint32_t x = 0;
                uint32_t y = 0;
                _data.get_nearest_index(point, x, y);
                candidates.emplace_back(x,y);
            }
        }

        void generate_sampling_points(const Triangle& meshTri, std::vector<Vector3d>& points) {
        }

        std::vector<std::array<double, 2>> generate_poisson_points(float radius, float max_x, float max_y, float min_x, float min_y) {

        }

        void try_deform(const Triangle& meshTri, const std::vector<std::pair<uint32_t, uint32_t>>& idx_v, double w) {
            for(const auto& point : idx_v) {
                auto v3 = _data.get_vertex_at_index(point.first, point.second);
                if(penetrates(meshTri, v3, w)) {
                    // Terra mx algo
                }
            }
        }

        bool penetrates(Triangle meshTri, Vector3d point, double w) {
            if(intersects_projected(std::move(meshTri), { point, w })) {
                // Height check
                return true;
            }
            return false;
        }

        bool intersects_projected(Triangle meshTri, AABB vertexRect) {
            return CGALGeometry::intersects_projected(std::move(meshTri), std::move(vertexRect));
        }

    };
}

#endif