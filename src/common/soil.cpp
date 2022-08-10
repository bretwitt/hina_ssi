#include "soil.h"
#include <cmath>

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

            _data->set_vertex_at_index(i, j, new VertexAttributes(v3));
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

    auto max_x = fmax(meshTri.v1.X(), fmax(meshTri.v2.X(), meshTri.v3.X()));
    auto max_y = fmax(meshTri.v1.Y(), fmax(meshTri.v2.Y(), meshTri.v3.Y()));
    auto min_x = fmin(meshTri.v1.X(), fmin(meshTri.v2.X(), meshTri.v3.X()));
    auto min_y = fmin(meshTri.v1.Y(), fmin(meshTri.v2.Y(), meshTri.v3.Y()));

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

bool Soil::penetrates(const Triangle& meshTri, VertexAttributes* vtx, double w) {
    auto point = vtx->v3;
    return (meshTri.centroid().Z() <= point.Z() && intersects_projected(meshTri, AABB(point, w )));
}

bool Soil::intersects_projected(const Triangle& meshTri, const AABB& vertexRect) {
    return Geometry::getInstance()->intersects_box_tri(meshTri, vertexRect) ;
}


void Soil::terramx_deform(const physics::LinkPtr& linkPtr, const Triangle& meshTri, uint32_t x, uint32_t y, VertexAttributes* vertex, double w, float dt) {
    auto v3 = vertex->v3;
    auto soil_z = v3.Z();
    auto mesh_z = meshTri.centroid().Z();

    auto rho = 2.0f;
    auto compress_modulus = 1000.0f;
    auto damp_coeff = rho*compress_modulus;

    auto k_phi = vertex->k_phi;

    auto sigma_t = k_phi*(soil_z-mesh_z);
    auto sigma_j = k_phi*(-mesh_z);

    if(sigma_t > 0) {
        auto dA = w * w;

        auto vtx_ul = _data->get_vertex_at_index(x - 1, y + 1)->v3;
        auto vtx_dl = _data->get_vertex_at_index(x - 1, y - 1)->v3;
        auto vtx_ur = _data->get_vertex_at_index(x + 1, y + 1)->v3;
        auto vtx_dr = _data->get_vertex_at_index(x + 1, y - 1)->v3;

        auto vtx_u = _data->get_vertex_at_index(x, y + 1)->v3;
        auto vtx_d = _data->get_vertex_at_index(x, y - 1)->v3;
        auto vtx_l = _data->get_vertex_at_index(x - 1, y)->v3;
        auto vtx_r = _data->get_vertex_at_index(x + 1, y)->v3;
        auto vtx = v3;

        auto tri1 = Triangle(vtx, vtx_ul, vtx_u);
        auto tri2 = Triangle(vtx, vtx_l, vtx_ul);
        auto tri3 = Triangle(vtx, vtx_u, vtx_r);

        auto tri4 = Triangle(vtx, vtx_r, vtx_d);
        auto tri5 = Triangle(vtx, vtx_dr, vtx_d);

        auto tri6 = Triangle(vtx, vtx_d, vtx_l);

        auto sum = tri1.normal() + tri2.normal() + tri3.normal() + tri4.normal() + tri5.normal() + tri6.normal();

        auto normal_force = ((sigma_j + damp_coeff) * dA * -sum.Normalize());
        auto normal_force_z = Vector3d( 0, 0, normal_force.Z());

//        auto c = 0.8;
//        auto phi = 0.645772;
//        auto tau_max = c + ((sigma_j + damp_coeff) * tan(phi) * dA * -sum.Normalize());
//        auto tau = Vector3d( tau_max.X(), tau_max.Y(), 0);

        //apply_force(linkPtr, v3, tau, dt);
        apply_force( linkPtr, v3, normal_force_z, dt);

        auto plastic_flow = -(soil_z - mesh_z) / 60.0f;
        auto _v3 = Vector3d(v3.X(), v3.Y(), soil_z + plastic_flow);
        vertex->v3 = _v3;
        _data->set_vertex_at_index(x, y, vertex);
    }
}

void Soil::apply_force(const physics::LinkPtr& linkPtr, const Vector3d& origin, const Vector3d& normal_force, float dt) {
    linkPtr->AddForceAtWorldPosition(normal_force, origin) ;
}