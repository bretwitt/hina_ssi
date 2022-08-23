#include "soil.h"
#include <cmath>
#include <omp.h>

using namespace gazebo;

Soil::Soil(SoilData* soil_data) {
    this->_data = soil_data;
    if (_data->soil_hashmap == nullptr) {
        _data->init_field();
        generate_geometry();
    }
}

Soil::~Soil()  {
    delete _data;
}

SoilData* Soil::get_data() {
    return _data;
}

void Soil::generate_geometry() {
    generate_soil_vertices();
    generate_indices();
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
    double max_x, max_y, min_x, min_y;
    double scale;
    int iter_x, iter_y;
    uint32_t x_start, y_start;

    //#pragma omp parallel default(none) shared(meshTri, link, _data) private(dt, max_x, max_y, min_x, min_y, iter_x, iter_y, x_start, y_start, scale)

    max_x = fmax(meshTri.v1.X(), fmax(meshTri.v2.X(), meshTri.v3.X()));
    max_y = fmax(meshTri.v1.Y(), fmax(meshTri.v2.Y(), meshTri.v3.Y()));
    min_x = fmin(meshTri.v1.X(), fmin(meshTri.v2.X(), meshTri.v3.X()));
    min_y = fmin(meshTri.v1.Y(), fmin(meshTri.v2.Y(), meshTri.v3.Y()));

    scale = _data->scale;
    iter_x = ceil( ((max_x - min_x) / scale) ) + 1;
    iter_y = ceil( ((max_y - min_y) / scale) ) + 1;

    x_start = 0;
    y_start = 0;

    _data->get_nearest_index(Vector2d(min_x, min_y), x_start, y_start);

    //#pragma omp parallel for collapse(2) num_threads(8)
    for(int x = 0; x < iter_x; x++) {
        for(int y = 0; y < iter_y; y++) {
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

void Soil::pre_update() {
    get_data()->sigma_tot = 0;
}

void Soil::terramx_deform(const physics::LinkPtr& linkPtr, const Triangle& meshTri, uint32_t x, uint32_t y, VertexAttributes* vertex, double w, float dt) {
    auto v3 = vertex->v3;
    auto s_p = -v3.Z();
    auto y_z = meshTri.centroid().Z();

    auto k_phi = vertex->k_phi;

    double s_y = -y_z;

    auto sigma_t = k_phi*(s_y - s_p);
    auto sigma_p = k_phi*(s_y);

    if(sigma_t > 0) { // Unilateral Contact

        /* Geometry Calculations */
        auto vtx_ul = _data->get_vertex_at_index(x - 1, y + 1)->v3;
        auto vtx_dl = _data->get_vertex_at_index(x - 1, y - 1)->v3;
        auto vtx_ur = _data->get_vertex_at_index(x + 1, y + 1)->v3;
        auto vtx_dr = _data->get_vertex_at_index(x + 1, y - 1)->v3;

        auto vtx_u = _data->get_vertex_at_index(x, y + 1)->v3;
        auto vtx_d = _data->get_vertex_at_index(x, y - 1)->v3;
        auto vtx_l = _data->get_vertex_at_index(x - 1, y)->v3;
        auto vtx_r = _data->get_vertex_at_index(x + 1, y)->v3;
        const auto& vtx = v3;

        auto tri1 = Triangle(vtx, vtx_ul, vtx_u);
        auto tri2 = Triangle(vtx, vtx_l, vtx_ul);
        auto tri3 = Triangle(vtx, vtx_u, vtx_r);

        auto tri4 = Triangle(vtx, vtx_r, vtx_d);
        auto tri5 = Triangle(vtx, vtx_dr, vtx_d);

        auto tri6 = Triangle(vtx, vtx_d, vtx_l);

        auto normal_sum = (tri1.normal() + tri2.normal() + tri3.normal() + tri4.normal() + tri5.normal() + tri6.normal()).Normalized();
        //auto area = (tri1.area() + tri2.area() + tri3.area() + tri4.area() + tri5.area() + tri6.area()) / 3;

        /* --- Stress --- */

        // Calculate force
        auto normal_dA = -normal_sum * w * w;

        auto r = linkPtr->WorldCoGPose().Pos() - v3;

        /* Friction */
        auto link_linear_vel = linkPtr->RelativeLinearVel();
        auto link_angular_vel = linkPtr->RelativeAngularVel();

        auto vertex_tangent_vel = link_linear_vel + ((r.Length())*(link_angular_vel));
        auto vtx_tangent = -(vertex_tangent_vel).Normalized()*w*w;
        auto friction_v = 0.8*sigma_p*vtx_tangent;

        /* Shear Effects, Normal Pressure, Friction */
        auto force_v = Vector3d((get_data()->c + (sigma_p * tan(get_data()->phi))) * normal_dA.X() * 0,
                                (get_data()->c + (sigma_p * tan(get_data()->phi))) * normal_dA.Y() * 0,
                                sigma_p * normal_dA.Z());// + friction_v;


        // Calculate torque
        auto torque_v = r.Cross(force_v);

        // Apply f/t
        linkPtr->AddForceAtWorldPosition(force_v, v3);
        //linkPtr->AddRelativeTorque(torque_v);

        /* --- Plastic Flow --- */
        auto plastic_flow = -(s_y - s_p)*dt;
        vertex->ds_p = plastic_flow;
        vertex->v3 = Vector3d(v3.X(), v3.Y(), v3.Z() + vertex->ds_p);

    } else {
        vertex->ds_p = 0;
    }

}


void Soil::apply_force(const physics::LinkPtr& linkPtr, const Vector3d& origin, const Vector3d& normal_force) {
    linkPtr->AddForceAtWorldPosition(normal_force, origin) ;
}
