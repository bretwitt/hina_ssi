#include "soil.h"

using namespace gazebo;

Soil::Soil(SoilData* soil_data) {
    this->_data = soil_data;
    if (_data->soil_hashmap == nullptr) {
        _data->init_field();
        generate_soil_vertices();
        generate_indices();
    }
}

Soil::~Soil()  {
    delete[] _data->soil_hashmap;
    delete[] _data->indices;
    delete _data;
}

SoilData* Soil::get_data() {
    return _data;
}

void Soil::generate_soil_vertices() {
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

void Soil::generate_indices() const {
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

void Soil::try_deform(const Triangle& meshTri, const physics::LinkPtr& link, float dt) {
    footprint_hash_idx_lookup_and_terramx_deform(meshTri, link, dt);
}

void Soil::footprint_hash_idx_lookup_and_terramx_deform(const Triangle& meshTri, const physics::LinkPtr& link, float dt) {
    auto max_x = std::max(meshTri.v1.X(), std::max(meshTri.v2.X(), meshTri.v3.X()));
    auto max_y = std::max(meshTri.v1.Y(), std::max(meshTri.v2.Y(), meshTri.v3.Y()));
    auto min_x = std::min(meshTri.v1.X(), std::min(meshTri.v2.X(), meshTri.v3.X()));
    auto min_y = std::min(meshTri.v1.Y(), std::min(meshTri.v2.Y(), meshTri.v3.Y()));

    auto scale = _data->scale;

    auto iter_x = ceil( ((max_x - min_x) / scale) ) + 1;
    auto iter_y = ceil( ((max_y - min_y) / scale) ) + 1;
    uint32_t x_start = 0;
    uint32_t y_start = 0;

    _data->get_nearest_index(Vector2d(min_x, min_y), x_start, y_start);

    for(uint32_t y = 0; y < iter_y; y++) {
        for(uint32_t x = 0; x < iter_x; x++) {
            auto v3 = _data->get_vertex_at_index(x + x_start, y + y_start);
            if(penetrates(meshTri, v3, scale)) {
                terramx_deform(link, meshTri, x + x_start, y + y_start, v3, scale, dt);
            }
        }
    }
}

bool Soil::penetrates(const Triangle& meshTri, const Vector3d& point, double w) {
    return (meshTri.centroid().Z() <= point.Z() && intersects_projected(meshTri, AABB(point, w )));
}

bool Soil::intersects_projected(const Triangle& meshTri, const AABB& vertexRect) {
    return Geometry::getInstance()->intersects_box_tri(meshTri, vertexRect) ;
}

void Soil::terramx_deform(const physics::LinkPtr& linkPtr, const Triangle& meshTri, uint32_t x, uint32_t y, Vector3d v3, double w, float dt) {
    auto soil_z = v3.Z();
    auto mesh_z = meshTri.centroid().Z();

    auto rho = 2.0f;
    auto compress_modulus = 1000.0f;
    auto damp_coeff = rho*compress_modulus;

    auto k_phi = 814000.0f;
    auto sigma = k_phi*(-mesh_z) + damp_coeff;

    if(sigma > 0) {
        auto dA = w * w;

        auto vtx_ul = _data->get_vertex_at_index(x - 1, y + 1);
        auto vtx_dl = _data->get_vertex_at_index(x - 1, y - 1);
        auto vtx_ur = _data->get_vertex_at_index(x + 1, y + 1);
        auto vtx_dr = _data->get_vertex_at_index(x + 1, y - 1);

        auto vtx_u = _data->get_vertex_at_index(x, y + 1);
        auto vtx_d = _data->get_vertex_at_index(x, y - 1);
        auto vtx_l = _data->get_vertex_at_index(x - 1, y);
        auto vtx_r = _data->get_vertex_at_index(x + 1, y);
        auto vtx = v3;

        auto tri1 = Triangle(vtx, vtx_ul, vtx_u);
        auto tri2 = Triangle(vtx, vtx_l, vtx_ul);
        auto tri3 = Triangle(vtx, vtx_u, vtx_r);

        auto tri4 = Triangle(vtx, vtx_r, vtx_d);
        auto tri5 = Triangle(vtx, vtx_dr, vtx_d);

        auto tri6 = Triangle(vtx, vtx_d, vtx_l);

        auto vtx_normal =
                -(tri_normal(tri1)
                  + tri_normal(tri2)
                  + tri_normal(tri3)
                  + tri_normal(tri4)
                  + tri_normal(tri5)
                  + tri_normal(tri6))
                        .Normalize();


        auto normal_force = (sigma * dA * vtx_normal);

        apply_normal_force( linkPtr, v3, normal_force, dt);

        auto plastic_flow = -(soil_z - mesh_z);

        auto _v3 = Vector3d(v3.X(), v3.Y(), soil_z + plastic_flow);
        _data->set_vertex_at_index(x, y, _v3);
    }
}

Vector3d Soil::tri_normal(const Triangle& tri) {
    auto p0 = tri.v1;
    auto p1 = tri.v2;
    auto p2 = tri.v3;

    auto A = p1 - p0;
    auto B = p2 - p0;
    auto norm = A.Cross(B).Normalized();
    return norm;
}

void Soil::apply_normal_force(const physics::LinkPtr& linkPtr, const Vector3d& origin, const Vector3d& normal_force, float dt) {
    auto normal_force_z = Vector3d(0,0,normal_force.Z());
    linkPtr->AddForceAtWorldPosition(normal_force_z * dt, origin) ;
}