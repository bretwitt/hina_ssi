#include "soil.h"
#include <cmath>
#include "../../../thirdparty/PerlinNoise.h"

using namespace gazebo;

Soil::Soil(uint32_t width, uint32_t height, double scale) {
    this->_data = std::make_shared<SoilData>(width, height, scale);
    _data->init_field();
}

Soil::Soil(SandboxConfig config) : Soil(config.x_width, config.y_width, config.scale) {
    generate_sandbox_geometry(config);
}

Soil::Soil(std::shared_ptr<DEM> dem) : Soil(dem->n, dem->m, dem->scale) {
    load_dem_geometry(dem);
}


std::shared_ptr<SoilData> Soil::get_data() {
    return _data;
}

void Soil::generate_sandbox_geometry(SandboxConfig config) {
    generate_sandbox_soil_vertices(config);
    generate_indices();
}

void Soil::generate_sandbox_soil_vertices(SandboxConfig config) {
    _data->x_offset = -(double) (_data->x_width - 1) / 2;
    _data->y_offset = -(double) (_data->y_width - 1) / 2;

    for (int j = 0; j < _data->y_width; j++) {
        for (int i = 0; i < _data->x_width; i++) {
            auto i_f = (float) i;
            auto j_f = (float) j;

            auto x = _data->scale * (i_f + _data->x_offset);
            auto y = _data->scale * (j_f + _data->y_offset);

            const double z = y*tan(config.angle);

            auto v3 = Vector3d(x, y, z);

            _data->set_vertex_at_index(i, j, std::make_shared<VertexAttributes>(v3));
        }
    }
}

void Soil::generate_indices() const {
    uint32_t x_size = _data->x_width;
    uint32_t y_size = _data->y_width;
    //auto indices = _data->indices;

    uint32_t idx = 0;
    for (uint32_t x = 0; x < x_size - 1; x++) {
        for (uint32_t y = 0; y < y_size - 1; y++) {
            uint32_t a = (x_size * y) + x;
            uint32_t b = (x_size * (y + 1)) + x;
            uint32_t c = (x_size * (y + 1)) + (x + 1);
            uint32_t d = (x_size * y) + (x + 1);

            _data->indices[idx++] = a;
            _data->indices[idx++] = d;
            _data->indices[idx++] = c;

            _data->indices[idx++] = c;
            _data->indices[idx++] = b;
            _data->indices[idx++] = a;
        }
    }
}

void Soil::load_dem_geometry(const std::shared_ptr<DEM>& dem) const {
    for(int i = 0; i < dem->m; i++) {
        for(int j = 0; j < dem->n; j++) {
            Vector3d dem_v3 = dem->soil_vertices[i][j]->v3;
            Vector3d v3 ( dem_v3.X(), dem_v3.Y(), dem_v3.Z());
            _data->set_vertex_at_index(i, j, std::make_shared<VertexAttributes>(v3));
        }
    }
    generate_indices();
}

std::vector<std::tuple<uint32_t, uint32_t, std::shared_ptr<VertexAttributes>>> Soil::try_deform(const Triangle& meshTri, const physics::LinkPtr& link, float dt, float& displaced_volume) {
    double max_x, max_y, min_x, min_y;
    double scale;
    int iter_x, iter_y;
    uint32_t x_start, y_start;

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

    std::vector<std::tuple<uint32_t, uint32_t, std::shared_ptr<VertexAttributes>>> penetrating_coords;

    for(uint32_t k = 0; k < iter_x*iter_y; k++) {
        uint32_t y = floor(k / iter_y);
        uint32_t x = k - (iter_x*y);
        auto v3 = _data->get_vertex_at_index(x + x_start, y + y_start);
        if(penetrates(meshTri, v3, scale)) {
            penetrating_coords.emplace_back(x + x_start,y + y_start,v3);
        }
    }

    // TODO: CUDA this
    for(auto & penetrating_coord : penetrating_coords) {
        auto x = std::get<0>(penetrating_coord);
        auto y = std::get<1>(penetrating_coord);
        auto v3 = std::get<2>(penetrating_coord);

        float displaced_volume_vtx = 0.0f;

        terramx_deform(link, meshTri, x, y, v3, scale, dt, displaced_volume_vtx);
        displaced_volume += displaced_volume_vtx;
    }
    return penetrating_coords;
}


bool Soil::penetrates(const Triangle& meshTri, const std::shared_ptr<VertexAttributes>& vtx, double w) {
    auto point = vtx->v3;
    return (meshTri.centroid().Z() <= point.Z() && intersects_projected(meshTri, AABB(point, w )));
}

bool Soil::intersects_projected(const Triangle& meshTri, const AABB& vertexRect) {
    return Geometry::getInstance()->intersects_box_tri(meshTri, vertexRect) ;
}


void Soil::terramx_deform(const physics::LinkPtr& linkPtr, const Triangle& meshTri, uint32_t x, uint32_t y, const std::shared_ptr<VertexAttributes>& vertex, double w, float dt, float& displaced_volume) {

    auto v3 = vertex->v3;
    auto v3_0 = vertex->v3_0;

    double k_phi = vertex->k_phi;
    double k_e = vertex->k_e;

    double y_h = meshTri.centroid().Z();
    double y_r = v3_0.Z();

    double s_y = y_r - y_h;
    double s_p = vertex->s_p;

    double sigma_t = k_e*(s_y - s_p);
    double sigma_p = 0.0;

    Vector3d normal_dA(0,0,0);

    auto s_sink = 0.0;

    if(sigma_t > 0) {                            // Unilateral Contact
        /* Geometry Calculations */
        auto vtx_ul = _data->get_vertex_at_index(x - 1, y + 1)->v3;
        auto vtx_dl = _data->get_vertex_at_index(x - 1, y - 1)->v3;
        auto vtx_ur = _data->get_vertex_at_index(x + 1, y + 1)->v3;
        auto vtx_dr = _data->get_vertex_at_index(x + 1, y - 1)->v3;

        auto vtx_u = _data->get_vertex_at_index(x, y + 1)->v3;
        auto vtx_d = _data->get_vertex_at_index(x, y - 1)->v3;
        auto vtx_l = _data->get_vertex_at_index(x - 1, y)->v3;
        auto vtx_r = _data->get_vertex_at_index(x + 1, y)->v3;
        const auto &vtx = v3;

        auto tri1 = Triangle(vtx, vtx_ul, vtx_u);
        auto tri2 = Triangle(vtx, vtx_l, vtx_ul);
        auto tri3 = Triangle(vtx, vtx_u, vtx_r);

        auto tri4 = Triangle(vtx, vtx_r, vtx_d);
        auto tri5 = Triangle(vtx, vtx_dr, vtx_d);

        auto tri6 = Triangle(vtx, vtx_d, vtx_l);

        auto normal_sum = (tri1.normal() + tri2.normal() + tri3.normal() + tri4.normal() + tri5.normal() +
                           tri6.normal()).Normalized();
        //auto area = (tri1.area() + tri2.area() + tri3.area() + tri4.area() + tri5.area() + tri6.area()) / 3;
        auto area = w * w;

        normal_dA = -normal_sum * area;

        vertex->normal_dA = normal_dA;


        auto sigma_star = sigma_t;
        s_sink = s_y;

        if(sigma_star < vertex->sigma_yield) {
            sigma_p = sigma_star;
        } else {
            sigma_p = (k_phi /* + (k_c/B)*/)*(s_y);
            vertex->sigma_yield = sigma_p;
            auto s_p_o = s_p;
            vertex->s_p = s_sink - (sigma_p / k_e);
            vertex->s_e = s_sink - s_p;
            vertex->plastic_flow = -(s_p - s_p_o)*dt;
        }
    }

    auto z = v3_0.Z() - s_p;
    auto force_origin = v3_0.Z() - s_sink;

    vertex->sigma = sigma_p;

    vertex->v3 = Vector3d(v3.X(), v3.Y(), z);


    auto force_v = Vector3d(
            (vertex->c + (vertex->sigma * tan(vertex->phi))) * vertex->normal_dA.X(),
            (vertex->c + (vertex->sigma * tan(vertex->phi))) * vertex->normal_dA.Y(),
            (vertex->sigma /*- (plastic_flow)*/) * vertex->normal_dA.Z()
    );

    // Apply f/t
    linkPtr->AddForceAtWorldPosition(force_v, Vector3d(v3_0.X(), v3_0.Y(), force_origin));

}

