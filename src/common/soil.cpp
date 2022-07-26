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

        void get_nearest_index(Vector2d vtx, uint32_t& x, uint32_t& y) const {
            auto x_unrounded = (vtx.X() - x_offset) / scale;
            auto y_unrounded = (vtx.Y() - x_offset) / scale;
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
            _data->x_offset = -(double) _data->x_width / 2;
            _data->y_offset = -(double) _data->y_width / 2;

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

        void try_deform(const Triangle& meshTri) {
            std::vector<std::pair<uint32_t, uint32_t>> idx_v;
            find_intersection_candidates(meshTri, idx_v);

            double w = 0.5;

            for(const auto& point : idx_v) {
                auto v3 = _data->get_vertex_at_index(point.first, point.second);
                if(penetrates(meshTri, v3, w)) {
                    terramx_deform(point.first, point.second, v3);
                }
            }
        }

        void find_intersection_candidates(const Triangle& meshTri, std::vector<std::pair<uint32_t, uint32_t>>& candidates) {
            _data->reset_collisions();

            std::vector<Vector2d> points;
            generate_sampling_points(meshTri,points);

            for(auto& point : points) {
                uint32_t x = 0;
                uint32_t y = 0;
                _data->get_nearest_index(point, x, y);
                candidates.emplace_back(x,y);
            }
        }

        void generate_sampling_points(const Triangle& meshTri, std::vector<Vector2d>& points) {
            std::unordered_map<double, std::unordered_map<double, int>> point_hashmap;

            auto av2 = Vector2d(meshTri.v1.X(), meshTri.v1.Y());
            auto bv2 = Vector2d(meshTri.v2.X(), meshTri.v2.Y());
            auto cv2 = Vector2d(meshTri.v3.X(), meshTri.v3.Y());

            if(av2 == bv2 || av2 == cv2 || bv2 == cv2) {    // Discard degenerate rectangles
                return;
            }

            for(uint32_t i = 0; i < 100; i++) {
                auto v2 = point_in_triangle( av2, bv2, cv2 );

                uint32_t x;
                uint32_t y;

                _data->get_nearest_index(v2, x, y);

                if(point_hashmap[x][y] != -1) {
                    point_hashmap[x][y] = -1;
                    points.push_back(v2);
                }
            }
        }

        static Vector2d point_in_triangle(Vector2d a, Vector2d b, Vector2d c) {
            // between [0, 1]
            auto s = (double)rand() / (double)RAND_MAX;
            auto t = (double)rand() / (double)RAND_MAX;

            if (s + t <= 1) {
                return s * (a + c) + t * (b + c);
            } else {
                return (1 - s) * (a + c) + (1 - t) * (b + c);
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
            return CGALGeometry::intersects_projected(std::move(meshTri), std::move(vertexRect));
        }

        void terramx_deform(uint32_t x, uint32_t y, Vector3d v3) {
            auto _v3 = Vector3d(v3.X(), v3.Y(), -1);
            _data->set_vertex_at_index(x, y, _v3);
        }


    };
}

#endif