#include "soil.h"

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

hina::Footprint Soil::try_deform(const TriangleContext &meshTri, const physics::LinkPtr &link) {


    auto idx = worldpos_to_chunk_idx(meshTri.tri.centroid());
    auto chunk = chunks->get_chunk({static_cast<int>(idx.X()),static_cast<int>(idx.Y())});

    if (chunk != nullptr && chunk->container != nullptr) {
        return chunk->container->try_deform(meshTri, link);
    }

    return {};
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Soil::compute_footprint_stage(const std::vector<Footprint>& footprint, const double& total_footprint_size, double& B) {

    double area = 0;
    double length = 0;
    int i = 0;
    int count = 0;

    struct pair {
        int i;
        int j;
        int x;
        int y;

        bool operator==(const pair& other) const {
            return i == other.i && j == other.j &&
                        x == other.x && y == other.y;
        }
    };

    struct pair_hash {
        std::size_t operator()(const pair& p) const {
            auto h1 = std::hash<int>{}(p.x);
            auto h2 = std::hash<int>{}(p.y);
            auto h3 = std::hash<int>{}(p.i);
            auto h4 = std::hash<int>{}(p.j);
            return h1 ^ h2 ^ h3 ^ h4;
        }
    };
    std::unordered_set<pair, pair_hash> visited;

    // Per mesh triangle in the footprint
    for (auto& triangle_ftp : footprint) {
        // Per vertex contact
        for(auto& contact : triangle_ftp.footprint) {


            // Count area by figuring out bordering nodes
            auto& x = contact.x;
            auto& y = contact.y;
            auto& ci = contact.i;
            auto& cj = contact.j;

            auto soil_chunk = chunks->get_chunk_cont(
                    ChunkedFieldLocation{contact.i,contact.j});
            auto field = soil_chunk->get_field();
            // Check for empty neighbors

            std::vector<pair> neighbors =
                    { { ci,cj, x + 1, y },
                      { ci,cj, x, y + 1 },
                      {ci,cj, x - 1, y},
                      {ci,cj, x, y - 1}
                    };

            for(auto& pair : neighbors) {
                auto& x = pair.x;
                auto& y = pair.y;

                if(visited.find(pair) != visited.end()) {
                    // this neighbor has already been visited
                    continue;
                }

                visited.insert(pair);

                auto vtx = field->get_vertex_at_index(x,y)->v;
                if(vtx->footprint == 2) { // Empty neighbor
                    count++;
                }
            }
            i++;
        }
    }

    length = count*0.005;
    area = 0.005*0.005*i;

    B = 2*area/length;
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

void Soil::query_chunk(const Vector3d& pos) {
    auto v2 = worldpos_to_chunk_idx(pos);
    chunks->poll_chunk({static_cast<int>(v2.X()),static_cast<int>(v2.Y())});
    chunks->poll_chunk({static_cast<int>(v2.X() + 1),static_cast<int>(v2.Y())});
    chunks->poll_chunk({static_cast<int>(v2.X() - 1),static_cast<int>(v2.Y())});
    chunks->poll_chunk({static_cast<int>(v2.X()),static_cast<int>(v2.Y()) + 1});
    chunks->poll_chunk({static_cast<int>(v2.X()),static_cast<int>(v2.Y()) - 1 });
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<SoilVertex> Soil::get_vertex_at_world_pos(Vector3d pos) {
//    auto cidx = worldpos_to_chunk_idx(pos);
//    chunks->get_chunk_cont()
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
    //chunks->post_update();
}


