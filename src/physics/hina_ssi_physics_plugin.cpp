#ifndef HINA_SSI_PHYSICS_PLUGIN_CPP
#define HINA_SSI_PHYSICS_PLUGIN_CPP

#define PHYS_PROFILER 0

#include <gazebo/common/common.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/common/Profiler.hh>
#include <memory>
#include <utility>

#include "soil/soil.h"
#include "dem/dem_loader.h"
#include "../common/field/uniform_field.h"

#include "SoilChunk.pb.h"

namespace hina {
    class HinaSSIWorldPlugin : public WorldPlugin {

    private:
        std::shared_ptr<Soil> soilPtr = nullptr;
        std::unique_ptr<msgs::Vector3d[]> soil_v = nullptr;
        std::unique_ptr<msgs::Vector2d[]> soil_id_v = nullptr;

        transport::NodePtr node = nullptr;
        transport::PublisherPtr soilPub = nullptr;
        event::ConnectionPtr updateEventPtr = nullptr;
        event::ConnectionPtr onEntityAddedEventPtr = nullptr;
        physics::WorldPtr world = nullptr;
        sdf::ElementPtr sdf = nullptr;

        Vector3d v3;
        std::map<physics::LinkPtr, const common::Mesh *> mesh_lookup{};

        common::Time time;
        double sec{};
        double last_sec{};
        double last_sec_viz{};
        int col_threads = 3;

    public:

        void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override {
            updateEventPtr = event::Events::ConnectBeforePhysicsUpdate(boost::bind(&HinaSSIWorldPlugin::update, this));
            onEntityAddedEventPtr = event::Events::ConnectAddEntity(
                    boost::bind(&HinaSSIWorldPlugin::OnEntityAdded, this, _1));
            world = _world;
            sdf = _sdf;
            init_threading();
            init_soil();
            init_transport();
            init_models();
        }

        void init_threading() {
            if (sdf->HasElement("col_threads")) {
                col_threads = sdf->GetElement("col_threads")->Get<int>();
            }
        }

        void OnEntityAdded(const std::string &str) {
            init_links(str);
        }

        void init_models() {
            auto model_v = world->Models();
            for (const auto &model: model_v) {
                init_links(model->GetName());
            }
        }

        void init_links(const std::string &model_name) {

            std::vector<std::string> links;
            auto link_element_ptr = sdf->GetElement("links");
            if (link_element_ptr != nullptr) {
                link_element_ptr = link_element_ptr->GetFirstElement();
                if (link_element_ptr != nullptr) {
                    links.push_back(link_element_ptr->Get<std::string>());
                }
                while (link_element_ptr != nullptr) {
                    link_element_ptr = link_element_ptr->GetNextElement("link");
                    if (link_element_ptr != nullptr) {
                        links.push_back(link_element_ptr->Get<std::string>());
                    }
                }
            }

            auto model = world->EntityByName(model_name)->GetParentModel();
            auto link_v = model->GetLinks();
            for (const auto &link: link_v) {

                if (std::end(links) == std::find(std::begin(links), std::end(links), link->GetName())) continue;

                auto collElemPtr = link->GetSDF()->GetElement("collision");
                if (collElemPtr != nullptr) {
                    if (!collElemPtr->HasElement("geometry")) break;
                    auto geomElemPtr = collElemPtr->GetElement("geometry");
                    if (geomElemPtr != nullptr) {
                        auto meshElemPtr = geomElemPtr->GetElement("mesh");
                        if (!geomElemPtr->HasElement("mesh")) break;
                        if (meshElemPtr != nullptr) {
                            auto uriElemPtr = meshElemPtr->GetElement("uri");
                            if (!meshElemPtr->HasElement("uri")) break;
                            if (uriElemPtr != nullptr) {
                                auto model_name = model->GetName();
                                auto uri = uriElemPtr->Get<std::string>();
                                load_mesh(link, uri);

                                std::cout << "Loaded " << model_name << ": " << link->GetName() << " uri..." << uri << std::endl;
                            }
                        }
                    }
                }
            }
        }

        void init_soil() {
            if (sdf->HasElement("dem")) {
                init_dem();
            } else if(sdf->HasElement("sandbox")){
                init_sandbox();
            }
        }

        void init_sandbox() {
            auto params = sdf->GetElement("params");

            auto k_phi = params->GetElement("k_phi")->Get<double>();
            auto k_e = params->GetElement("k_e")->Get<double>();
            auto c = params->GetElement("c")->Get<double>();
            auto phi = params->GetElement("phi")->Get<double>();
            auto mfr = params->GetElement("mfr")->Get<double>();

            auto soil_params = SoilPhysicsParams { k_phi, k_e, 0, c, phi, mfr };

            auto sandbox_elem = sdf->GetElement("sandbox");
            auto x_width = sandbox_elem->GetElement("width_x")->Get<double>();
            auto y_width = sandbox_elem->GetElement("width_y")->Get<double>();
            auto scale = sandbox_elem->GetElement("resolution")->Get<double>();
            auto angle = sandbox_elem->GetElement("angle")->Get<double>();

            auto sandbox_sampler = std::make_shared<SandboxVertexSampler>(angle,soil_params);
            soilPtr = std::make_shared<Soil>(sandbox_sampler, FieldTrueDimensions { x_width, y_width }, scale);
        }

        void init_dem() {
            auto dem_elem = sdf->GetElement("dem");

            std::string filename = dem_elem->GetElement("file")->Get<std::string>();
            std::string file_name = gazebo::common::SystemPaths::Instance()->FindFile(filename);
            auto dem = DEMLoader::load_dem_from_geotiff(file_name);

            if(dem_elem->HasElement("upscale_res")) {
                auto upscale_res = dem_elem->GetElement("upscale_res")->Get<double>();
                dem->upsample(upscale_res);
            }

            auto sampler = std::make_shared<DEMVertexSampler>(dem);
            soilPtr = std::make_shared<Soil>(sampler,FieldTrueDimensions { 0.5, 0.5 }, dem->field->scale);
        }

        void init_transport() {
            this->node = transport::NodePtr(new transport::Node());
            node->Init();
            soilPub = node->Advertise<hina_ssi_msgs::msgs::SoilChunk>("~/soil");
        }

        void load_mesh(const physics::LinkPtr &link, const std::string &mesh_uri) {
            auto mesh = common::MeshManager::Instance()->Load(mesh_uri);
            mesh_lookup.insert(std::pair<physics::LinkPtr, const common::Mesh *>(link, mesh));
        }

        void update() {

            time = common::Time::GetWallTime();
            sec = time.Double();

#ifdef PHYS_PROFILER
            IGN_PROFILE_BEGIN("TAROSCM::Soil Physics Update");
#endif
            double dt = sec - last_sec;
            double dt_viz = sec - last_sec_viz;

            update_soil(soilPtr, dt);

#ifdef PHYS_PROFILER
            IGN_PROFILE_END();
            IGN_PROFILE_BEGIN("TAROSCM::Broadcast Soil");
#endif

            last_sec = sec;

            if (dt_viz > (1. / 1.f)) {
                broadcast_soil(soilPtr);
                last_sec_viz = sec;
            }

#ifdef PHYS_PROFILER
            IGN_PROFILE_END();
#endif

        }


        void update_soil(std::shared_ptr<Soil> soil, double dt) {

            soil->pre_update();

            for (auto &iter: mesh_lookup) {
                auto link = iter.first;
                auto mesh = iter.second;

                for (uint32_t i = 0; i < mesh->GetSubMeshCount(); i++) {

                    auto submesh = mesh->GetSubMesh(i);
                    auto indices = submesh->GetIndexCount();

                    auto col_pose = link->GetCollision(0.)->RelativePose();
                    auto cpos = col_pose.Pos();
                    auto crot = col_pose.Rot();

                    auto pose = link->WorldPose();
                    auto rot = pose.Rot();
                    auto pos = pose.Pos();

                    auto aabb = link->GetCollision(0.)->BoundingBox();
                    auto max = aabb.Max();
                    auto min = aabb.Min();

                    soil->query_chunk((aabb.Max() + aabb.Min()) / 2);

                    std::vector<std::vector<std::tuple<uint32_t, uint32_t, SoilChunk, std::shared_ptr<FieldVertex<SoilVertex>>>>> footprint;
                    std::vector<std::tuple<uint32_t, uint32_t, SoilChunk, std::shared_ptr<FieldVertex<SoilVertex>>>> footprint_idx;

                    double total_displaced_volume = 0.0f;
                    float total_footprint_size = 0.0f;

                    // Vertex level computations
#ifdef PHYS_PROFILER
                    IGN_PROFILE_BEGIN("Collision Detection");
#endif
                    #pragma omp parallel num_threads(col_threads) default(none) shared(footprint, total_footprint_size, total_displaced_volume) firstprivate(footprint_idx, submesh, soil, crot, cpos, pos, rot, indices, dt, link)
                    {
                        #pragma omp for nowait schedule(guided) reduction(+:total_displaced_volume)

                         for (uint32_t idx_unrolled = 0; idx_unrolled < (indices / 3); idx_unrolled++) {

                            auto idx = idx_unrolled * 3;

                            auto v0 = submesh->Vertex(submesh->GetIndex(idx));
                            auto v1 = submesh->Vertex(submesh->GetIndex(idx + 1));
                            auto v2 = submesh->Vertex(submesh->GetIndex(idx + 2));

                            auto cv0 = crot.RotateVector(v0) + cpos;
                            auto cv1 = crot.RotateVector(v1) + cpos;
                            auto cv2 = crot.RotateVector(v2) + cpos;

                            auto c1v0 = rot.RotateVector(cv0) + pos;
                            auto c1v1 = rot.RotateVector(cv1) + pos;
                            auto c1v2 = rot.RotateVector(cv2) + pos;

                            auto meshTri = Triangle(c1v0, c1v1, c1v2);

                            double displaced_volume = 0.0f;
                            footprint_idx = soil->try_deform(meshTri, link, displaced_volume, dt);

                            total_displaced_volume += displaced_volume;

                            #pragma omp critical
                            {
                                footprint.push_back(footprint_idx);
                            }
                        }
                    }


#ifdef PHYS_PROFILER
                    IGN_PROFILE_END();
#endif


                    // Footprint level computations

                    // 1. Compute border and inner nodes
                    // 2. Compute footprint size

                    double footprint_size;

                    std::unordered_map<uint32_t, std::unordered_map<uint32_t,
                            std::unordered_map<uint32_t, std::unordered_map<uint32_t,
                                    int>>>> deposit_footprint;

                    std::vector<std::tuple<SoilChunk, uint32_t, uint32_t, std::shared_ptr<FieldVertex<SoilVertex>>>> deposit_footprint_v;

                    for (const auto &tri_ftp: footprint) {
                        for (const auto &vtx: tri_ftp) {
                            auto x = std::get<0>(vtx);
                            auto y = std::get<1>(vtx);
                            auto chunk = std::get<2>(vtx);

                            auto field = chunk.field;

                            auto vtx1 = field->get_vertex_at_index(x + 1, y);
                            auto vtx2 = field->get_vertex_at_index(x, y + 1);
                            auto vtx3 = field->get_vertex_at_index(x - 1, y);
                            auto vtx4 = field->get_vertex_at_index(x, y - 1);

                            if (vtx1->v->footprint == 2 &&
                                (deposit_footprint[chunk.location.i][chunk.location.j][x + 1][y] == 0)) {
                                deposit_footprint[chunk.location.i][chunk.location.j][x + 1][y] = 2;
                                deposit_footprint_v.emplace_back(chunk, x + 1, y, vtx1);
                            }
                            if (vtx2->v->footprint == 2 &&
                                (deposit_footprint[chunk.location.i][chunk.location.j][x][y + 1] == 0)) {
                                deposit_footprint[chunk.location.i][chunk.location.j][x][y + 1] = 2;
                                deposit_footprint_v.emplace_back(chunk, x, y + 1, vtx2);
                            }
                            if (vtx3->v->footprint == 2 &&
                                (deposit_footprint[chunk.location.i][chunk.location.j][x - 1][y] == 0)) {
                                deposit_footprint[chunk.location.i][chunk.location.j][x - 1][y] = 2;
                                deposit_footprint_v.emplace_back(chunk, x - 1, y, vtx3);
                            }
                            if (vtx4->v->footprint == 2 &&
                                (deposit_footprint[chunk.location.i][chunk.location.j][x][y - 1] == 0)) {
                                deposit_footprint[chunk.location.i][chunk.location.j][x][y - 1] = 2;
                                deposit_footprint_v.emplace_back(chunk, x, y - 1, vtx4);
                            }
                        }
                    }
                    // 3. Deposit soil

                    double deposit = 0;
                    if (!footprint.empty()) {
                        deposit = total_displaced_volume / (double) deposit_footprint_v.size();
                    }


                    for (const auto &v: deposit_footprint_v) {
                        auto chunk = std::get<0>(v);
                        auto vtx = std::get<3>(v);
                        double w = chunk.field->scale;

                        if (vtx->v->footprint == 2) {
                            vtx->v3 += Vector3d(0, 0, deposit / (w * w));
                            if(vtx->v3.Z() >= chunk.field->scale/(tan(0.5))) {
                                vtx->v3 = Vector3d(vtx->v3.X(), vtx->v3.Y(), chunk.field->scale/tan(0.5));
                            }
                        }

                    }

                    // 4. Erode


/*                    std::vector<std::tuple<SoilChunk, uint32_t, uint32_t, std::shared_ptr<FieldVertex<SoilVertex>>>> flow_graph_v;
//
//                    for (const auto &v: deposit_footprint_v) {
//                        auto chunk = std::get<0>(v);
//                        auto x = std::get<1>(v);
//                        auto y = std::get<2>(v);
//                        auto vtx = std::get<3>(v);
//                        auto f = chunk.field;
//
//                        if (vtx->v->footprint == 2) {
//                            auto v1 = f->get_vertex_at_index(x + 1, y);
//                            auto v2 = f->get_vertex_at_index(x - 1, y);
//
//                            auto v3 = f->get_vertex_at_index(x, y + 1);
//                            auto v4 = f->get_vertex_at_index(x, y - 1);
//
//                            if (v1->v->footprint == 2 || v1->v->footprint == 0) {
//                                if (v1->v->footprint == 0) {
//                                    v1->v->footprint = 3;
//                                }
//                                if (deposit_footprint[chunk.location.i][chunk.location.j][x + 1][y] == 0) {
//                                    deposit_footprint[chunk.location.i][chunk.location.j][x + 1][y] = 1;
//                                    deposit_footprint_v.emplace_back(chunk, x + 1, y, v1);
//                                    flow_graph_v.emplace_back(chunk, x + 1, y, v1);
//                                }
//                                if (deposit_footprint[chunk.location.i][chunk.location.j][x - 1][y] == 0) {
//                                    deposit_footprint[chunk.location.i][chunk.location.j][x - 1][y] = 1;
//                                    deposit_footprint_v.emplace_back(chunk, x - 1, y, v1);
//                                    flow_graph_v.emplace_back(chunk, x - 1, y, v1);
//                                }
//                                if (deposit_footprint[chunk.location.i][chunk.location.j][x][y + 1] == 0) {
//                                    deposit_footprint[chunk.location.i][chunk.location.j][x][y + 1] = 1;
//                                    deposit_footprint_v.emplace_back(chunk, x, y + 1, v1);
//                                    flow_graph_v.emplace_back(chunk, x, y + 1, v1);
//                                }
//                                if (deposit_footprint[chunk.location.i][chunk.location.j][x][y - 1] == 0) {
//                                    deposit_footprint[chunk.location.i][chunk.location.j][x][y - 1] = 1;
//                                    deposit_footprint_v.emplace_back(chunk, x, y - 1, v1);
//                                    flow_graph_v.emplace_back(chunk, x, y - 1, v1);
//                                }
//                            }
//                        }
//                  }
 */

/*
//                    for(uint32_t i = 0; i < 6; i++) {
//                        for(const auto& v : deposit_footprint_v) {
//                            auto chunk = std::get<0>(v);
//                            auto x = std::get<1>(v);
//                            auto y = std::get<2>(v);
//                            auto vtx = std::get<3>(v);
//                            auto z = vtx->v3.Z();
//
//                            auto f = chunk.field;
//
//                            if(vtx->v->footprint == 2) {
//                                if(z >= f->scale/(tan(0.5))) {
//                                    auto v1 = f->get_vertex_at_index(x+1,y);
//                                    auto v2 = f->get_vertex_at_index(x,y+1);
//                                    auto v3 = f->get_vertex_at_index(x-1,y);
//                                    auto v4 = f->get_vertex_at_index(x,y-1);
//
//                                    uint32_t j = 0;
//                                    if(v1->v->footprint == 2 || v1->v->footprint == 0) {
//                                        j++;
//                                    } else if (v2->v->footprint == 2 || v2->v->footprint == 0) {
//                                        j++;
//                                    } else if (v3->v->footprint == 2 || v3->v->footprint == 0) {
//                                        j++;
//                                    } else if (v4->v->footprint == 2 || v4->v->footprint == 0) {
//                                        j++;
//                                    }
//
//                                    auto flow_out = 0.5*z*(1/(float)j);
//                                    if(v1->v->footprint == 2 || v1->v->footprint == 0) {
//                                        v1->v3 += Vector3d(0 ,0, flow_out);
//                                        v1->v->footprint = 2;
//                                        if(deposit_footprint[chunk.location.i][chunk.location.j][x+1][y] == 0) {
//                                            deposit_footprint[chunk.location.i][chunk.location.j][x+1][y] = 1;
//                                            deposit_footprint_v.emplace_back(chunk, x, y-1, v1);
//                                        }
//                                    } else if (v2->v->footprint == 2 || v2->v->footprint == 0) {
//                                        v2->v3 += Vector3d(0 ,0, flow_out);
//                                        v2->v->footprint = 2;
//                                        if(deposit_footprint[chunk.location.i][chunk.location.j][x][y+1] == 0) {
//                                            deposit_footprint[chunk.location.i][chunk.location.j][x][y+1] = 1;
//                                            deposit_footprint_v.emplace_back(chunk, x, y+1, v1);
//                                        }
//                                    } else if (v3->v->footprint == 2 || v3->v->footprint == 0) {
//                                        v3->v3 += Vector3d(0 ,0, flow_out);
//                                        v3->v->footprint = 2;
//                                        if(deposit_footprint[chunk.location.i][chunk.location.j][x-1][y] == 0) {
//                                            deposit_footprint[chunk.location.i][chunk.location.j][x-1][y] = 1;
//                                            deposit_footprint_v.emplace_back(chunk, x-1, y, v1);
//                                        }
//                                    } else if (v4->v->footprint == 2 || v4->v->footprint == 0) {
//                                        v4->v3 += Vector3d(0 ,0, flow_out);
//                                        v4->v->footprint = 2;
//                                        if(deposit_footprint[chunk.location.i][chunk.location.j][x][y-1] == 0) {
//                                            deposit_footprint[chunk.location.i][chunk.location.j][x][y-1] = 1;
//                                            deposit_footprint_v.emplace_back(chunk, x, y-1, v1);
//                                        }
//                                    }
//                                    vtx->v3 -= Vector3d(0,0, z - flow_out);
//                                }
//
//                            }
//                        }
//                    }
                     */

                }

            }

            soil->post_update();

        }

        void broadcast_soil(const std::shared_ptr<Soil>& soilPtr) {

            auto chunks = soilPtr->get_chunks()->get_active_chunks();

            for(auto& chunk : chunks) {
                hina_ssi_msgs::msgs::SoilChunk chunk_update_msg;

                auto field = chunk->container->field;
                auto x_w = field->x_vert_width;
                auto y_w = field->y_vert_width;

                for(uint32_t i = 0; i < x_w*y_w; i++) {
                    auto update = chunk_update_msg.add_chunk_field();
                    auto v3 = field->get_vertex_at_flattened_index(i)->v3;
                    auto msg = gazebo::msgs::Vector3d();
                    msg.set_x(v3.X());
                    msg.set_y(v3.Y());
                    msg.set_z(v3.Z());
                    *update = msg;

                    chunk_update_msg.add_footprint_field(field->get_vertex_at_flattened_index(i)->v->footprint);

                }

                chunk_update_msg.set_len_row(x_w);
                chunk_update_msg.set_len_col(y_w);
                chunk_update_msg.set_id_i(chunk->location.i);
                chunk_update_msg.set_id_j(chunk->location.j);
                chunk_update_msg.set_len_col(y_w);

                soilPub->Publish(chunk_update_msg);

                chunk->container->clear_footprint();
            }

        }
    };
    GZ_REGISTER_WORLD_PLUGIN(HinaSSIWorldPlugin)
}
#endif
