#include "soil.h"
#include "../scm/terramechanics.h"

#include <utility>

using namespace gazebo;
using namespace hina;

using ignition::math::Vector3d;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Soil::Soil(std::shared_ptr<SoilVertexSampler> sampler, FieldTrueDimensions dims, double scale) : Soil(dims, scale) {
    this->sampler = std::move(sampler);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Soil::Soil(FieldVertexDimensions dims, double scale) : Soil() {
    this->scale = scale;
    vtx_dims = dims;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Soil::Soil(FieldTrueDimensions dims, double scale) : Soil() {
    this->scale = scale;
    vtx_dims = UniformField<SoilVertex>::as_vtx_dims(dims, scale);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Soil::Soil() {
    chunks = std::make_shared<ChunkedField<std::shared_ptr<SoilChunk>>>();
    chunks->register_chunk_create_callback(boost::bind(&Soil::OnChunkCreation, this, _1, _2));
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

hina::Soil::Field_V Soil::try_deform(const Triangle &meshTri, const physics::LinkPtr &link, double &displaced_volume) {
    return this->deform(meshTri, link, displaced_volume);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

hina::Soil::Field_V Soil::deform(const Triangle& meshTri, const physics::LinkPtr& link, double& displaced_vol) {

    double max_x, max_y, min_x, min_y;
    int iter_x, iter_y;
    int x_start, y_start;
    int x_end, y_end;

    // AABB bounds under mesh triangle
    max_x = fmax(meshTri.v1.X(), fmax(meshTri.v2.X(), meshTri.v3.X()));
    max_y = fmax(meshTri.v1.Y(), fmax(meshTri.v2.Y(), meshTri.v3.Y()));
    min_x = fmin(meshTri.v1.X(), fmin(meshTri.v2.X(), meshTri.v3.X()));
    min_y = fmin(meshTri.v1.Y(), fmin(meshTri.v2.Y(), meshTri.v3.Y()));

    // Start constructing bounds to search soil field

    // Get first soil vertex coordinates (x_start, y_start)
    x_start = 0;
    y_start = 0;
    x_end = 0;
    y_end = 0;

    get_nearest_gidx(Vector3d(min_x,min_y,0), x_start, y_start);
    get_nearest_gidx(Vector3d(max_x,max_y,0), x_end, y_end);

    iter_x = abs(x_end - x_start) + 1;
    iter_y = abs(y_end - y_start) + 1;


    // Search soil field within bounds under AABB
    hina::Soil::Field_V penetrating_coords;


    for (int y = 0; y < iter_y; y++) {
        for (int x = 0; x < iter_x; x++) {

            int x_ = static_cast<int>(x) + x_start;
            int y_ = static_cast<int>(y) + y_start;

            auto v3 = get_vertex_from_global(x_, y_);
            //v3->v->footprint = 1;

            // If mesh tri penetrates vtx

            if(Terramechanics::intersects_projected(meshTri, hina::AABB(v3->v3, 0.005))) {
                v3->v->footprint = 1;
            }

//            if (!v3->v->isAir && Terramechanics::penetrates(meshTri, v3, this->scale)) {
//                Vector3d force_v, v3_0, force_origin;
//                //penetrating_coords.emplace_back(x + x_start, y + y_start, *chunk, v3);
//
//                get_vertex_from_global(x_,y_)->v->footprint = 1;
//
//                auto vtx_ul = get_vertex_from_global(x_ - 1, y_ + 1);
//                auto vtx_dl = get_vertex_from_global(x_ - 1, y_ - 1);
//                auto vtx_ur = get_vertex_from_global(x_ + 1, y_ + 1);
//                auto vtx_dr = get_vertex_from_global(x_ + 1, y_ - 1);
//
//                auto vtx_u = get_vertex_from_global(x_, y_ + 1);
//                auto vtx_d = get_vertex_from_global(x_, y_ - 1);
//                auto vtx_l = get_vertex_from_global(x_ - 1, y_);
//                auto vtx_r = get_vertex_from_global(x_ + 1, y_);
//
//                hina::VtxNeighbors neighbors = {vtx_ul, vtx_dl, vtx_ur, vtx_dr, vtx_u, vtx_d, vtx_l, vtx_r};
//
//                Terramechanics::terramx_deform(meshTri, v3, this->scale, displaced_vol,
//                                               sampler->get_params_at_index(x_, y_),
//                                               neighbors, force_v, force_origin);
                // Apply f/t
//                link->AddForceAtWorldPosition(force_v, force_origin);
//            }
        }
    }

    return penetrating_coords;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Soil::compute_footprint_stage(const Footprint_V& footprint, double total_displaced_volume) {

    std::unordered_map<uint32_t, std::unordered_map<uint32_t,
                        std::unordered_map<uint32_t, std::unordered_map<uint32_t,int>>>> deposit_footprint;

    std::vector<std::tuple<SoilChunk, uint32_t, uint32_t,
                        std::shared_ptr<FieldVertex<SoilVertex>>>> deposit_footprint_v;
    // Footprint level computations

    // 1. Compute border and inner nodes
    // 2. Compute footprint size

    for (const auto &tri_ftp: footprint) {
        for (const auto &vtx: tri_ftp) {
            auto x = std::get<0>(vtx);
            auto y = std::get<1>(vtx);
            auto chunk = std::get<2>(vtx);

            auto field = chunk.get_field();
            auto loc = chunk.get_location();

            auto vtx1 = field->get_vertex_at_index(x + 1, y);
            auto vtx2 = field->get_vertex_at_index(x, y + 1);
            auto vtx3 = field->get_vertex_at_index(x - 1, y);
            auto vtx4 = field->get_vertex_at_index(x, y - 1);

            if (vtx1->v->footprint == 2 &&
                (deposit_footprint[loc.i][loc.j][x + 1][y] == 0)) {
                deposit_footprint[loc.i][loc.j][x + 1][y] = 2;
                deposit_footprint_v.emplace_back(chunk, x + 1, y, vtx1);
            }
            if (vtx2->v->footprint == 2 &&
                (deposit_footprint[loc.i][loc.j][x][y + 1] == 0)) {
                deposit_footprint[loc.i][loc.j][x][y + 1] = 2;
                deposit_footprint_v.emplace_back(chunk, x, y + 1, vtx2);
            }
            if (vtx3->v->footprint == 2 &&
                (deposit_footprint[loc.i][loc.j][x - 1][y] == 0)) {
                deposit_footprint[loc.i][loc.j][x - 1][y] = 2;
                deposit_footprint_v.emplace_back(chunk, x - 1, y, vtx3);
            }
            if (vtx4->v->footprint == 2 &&
                (deposit_footprint[loc.i][loc.j][x][y - 1] == 0)) {
                deposit_footprint[loc.i][loc.j][x][y - 1] = 2;
                deposit_footprint_v.emplace_back(chunk, x, y - 1, vtx4);
            }
        }
    }

    // 3. Deposit soil

    double deposit = 0;
    if (!footprint.empty()) {
        deposit = total_displaced_volume / (double) deposit_footprint_v.size();
    }

    Vector2d min;
    Vector2d max;

    for (const auto &v: deposit_footprint_v) {
        auto chunk = std::get<0>(v);
        auto vtx = std::get<3>(v);
        auto v3 = vtx->v3;

        double w = chunk.get_field()->scale;

        if(v3.X() > max.X()) {
            max = Vector2d(v3.X(), max.Y());
        }
        else if(v3.X() < min.X()) {
            min = Vector2d(v3.X(), min.Y());
        }
        if(v3.Y() > max.Y()) {
            max = Vector2d(max.X(), v3.Y());
        }
        else if(v3.Y() < min.Y()) {
            min = Vector2d(min.X(), v3.Y());
        }

        if (vtx->v->footprint == 2) {
            vtx->v3 += Vector3d(0, 0, deposit / (w * w));
//               if(vtx->v3.Z() >= w/(tan(0.5))) {
//                    vtx->v3 = Vector3d(vtx->v3.X(), vtx->v3.Y(), w/tan(0.5));
//               }
        }
    }

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Vector2d Soil::chunk_idx_to_worldpos(int i, int j) const {
    return { i*((this->vtx_dims.verts_x-1)*this->scale), j*((this->vtx_dims.verts_y-1)*this->scale) };
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Vector2d Soil::worldpos_to_chunk_idx(Vector3d pos) const {
    return { floor(pos.X() / (this->vtx_dims.verts_x*this->scale)),
             floor(pos.Y() / (this->vtx_dims.verts_y*this->scale))
    };
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Soil::get_global_idx(SoilChunkLocation loc, int x, int y, int& g_x, int& g_y) {
    g_x = x + (loc.origin.X()/scale);
    g_y = y + (loc.origin.Y()/scale);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Soil::get_nearest_gidx(Vector3d pos, int& g_x, int& g_y) const {
    g_x = floor(pos.X()/scale);
    g_y = floor(pos.Y()/scale);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<FieldVertex<SoilVertex>> Soil::get_vertex_from_global(int g_x, int g_y) {

    int c_origin_x = static_cast<int>(floor(g_x / static_cast<float>(this->vtx_dims.verts_x)));
    int c_origin_y = static_cast<int>(floor(g_y / static_cast<float>(this->vtx_dims.verts_y)));

    #pragma omp critical
    {
        chunks->poll_chunk(ChunkedFieldLocation { c_origin_x, c_origin_y });
    }

    auto chunk = chunks->get_chunk(ChunkedFieldLocation{ c_origin_x, c_origin_y });

    int res = static_cast<int>(vtx_dims.verts_x);

    int l_x = g_x % res;
    int l_y = g_y % res;

    auto l_x_u = static_cast<uint32_t>((l_x + res) % res);
    auto l_y_u = static_cast<uint32_t>((l_y + res) % res);

//    unsigned int l_x_u = (l_x < 0) ? res-abs(l_x) : l_x;
//    unsigned int l_y_u = (l_y < 0) ? res-abs(l_y) : l_y;

    if(chunk != nullptr && chunk->container != nullptr) {
        return chunk->container->get_field()->get_vertex_at_index(l_x_u,l_y_u);

    }
    return nullptr;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Soil::query_chunk(const Vector3d& pos) {
    auto v2 = worldpos_to_chunk_idx(pos);
    chunks->poll_chunk({static_cast<int>(v2.X()),static_cast<int>(v2.Y())});
    chunks->poll_chunk({static_cast<int>(v2.X() + 1),static_cast<int>(v2.Y())});
    chunks->poll_chunk({static_cast<int>(v2.X() - 1),static_cast<int>(v2.Y())});
    chunks->poll_chunk({static_cast<int>(v2.X()),static_cast<int>(v2.Y()) + 1});
    chunks->poll_chunk({static_cast<int>(v2.X()),static_cast<int>(v2.Y()) - 1 });
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<SoilChunk> Soil::OnChunkCreation(int i, int j) {
    auto sc = std::make_shared<SoilChunk>();
    std::cout << "Chunk Created: " << i << " " << j << std::endl;
    sc->init_chunk(vtx_dims, scale, { i,j, chunk_idx_to_worldpos(i,j)}, sampler);
    return sc;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<ChunkedField<std::shared_ptr<SoilChunk>>> Soil::get_chunks() {
    return this->chunks;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Soil::pre_update() {
    chunks->pre_update();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Soil::post_update() {
    chunks->post_update();
}


