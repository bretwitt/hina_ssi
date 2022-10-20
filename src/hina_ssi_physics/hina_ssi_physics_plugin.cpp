#ifndef HINA_SSI_PHYSICS_PLUGIN_CPP
#define HINA_SSI_PHYSICS_PLUGIN_CPP

#include <gazebo/common/common.hh>
#include <memory>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/physics/physics.hh>
#include <utility>
#include "../common/soil.h"
#include "Soil.pb.h"

namespace gazebo {
    class HinaSSIWorldPlugin : public WorldPlugin {

    private:
        Soil *soilPtr = nullptr;
        msgs::Vector3d *soil_v = nullptr;

        transport::NodePtr node = nullptr;
        transport::PublisherPtr soilPub = nullptr;
        event::ConnectionPtr updateEventPtr = nullptr;
        event::ConnectionPtr onEntityAddedEventPtr = nullptr;
        physics::WorldPtr world = nullptr;
        sdf::ElementPtr sdf = nullptr;

        double z{};
        Vector3d v3;
        std::map<physics::LinkPtr, const common::Mesh*> mesh_lookup {};

        common::Time time;
        double sec{};
        double last_sec{};
        double last_sec_viz{};

    public:
        HinaSSIWorldPlugin() : WorldPlugin() {
        }

        ~HinaSSIWorldPlugin() override {
            delete soilPtr;
            delete soil_v;
        }

        void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override {
            updateEventPtr = event::Events::ConnectBeforePhysicsUpdate(boost::bind(&HinaSSIWorldPlugin::update, this));
            onEntityAddedEventPtr = event::Events::ConnectAddEntity(boost::bind(&HinaSSIWorldPlugin::OnEntityAdded, this, _1));
            world = _world;
            sdf = _sdf;
            init_soil();
            init_transport();
            init_models();
            //load_meshes();
        }

        void OnEntityAdded(const std::string& str) {
//            std::string links[6] = { "wheel_RR_link", "wheel_FR_link", "wheel_RL_link", "wheel_FL_link", "wheel_R_link", "wheel_L_link" };
            init_links(str);
        }

        void init_models() {
            auto model_v = world->Models();
            for(const auto& model : model_v) {
                init_links(model->GetName());
            }
        }

        void init_links(std::string model_name) {
            std::string links[4] = { "FR_wheel_link","BR_wheel_link","FL_wheel_link","BL_wheel_link" };
            //std::string links[1] = { "wheel" };
            auto model = world->EntityByName(model_name)->GetParentModel();
            auto link_v = model->GetLinks();
            for(const auto& link : link_v) {

                if(std::end(links) == std::find(std::begin(links), std::end(links), link->GetName())) continue;

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
                            }
                        }
                    }
                }
            }
        }

        void init_soil() {
            soilPtr = new Soil(new SoilData (500,500,0.005f));
        }

        void init_transport() {
            auto df = soilPtr->get_data();
            soil_v = new msgs::Vector3d[df->x_width * df->y_width];
            this->node = transport::NodePtr(new transport::Node());
            node->Init();
            soilPub = node->Advertise<hina_ssi_msgs::msgs::Soil>("~/soil");
        }

        void load_mesh(const physics::LinkPtr& link, const std::string& mesh_uri) {
            auto mesh = common::MeshManager::Instance()->Load(mesh_uri);
            mesh_lookup.insert(std::pair<physics::LinkPtr, const common::Mesh*>(link, mesh));
        }

        void update() {
            time = common::Time::GetWallTime();
            sec = time.Double();

            double dt = sec - last_sec;
            double dt_viz = sec - last_sec_viz;

            update_soil(soilPtr, dt);

            last_sec = sec;

            if(dt_viz > (1./5.f)) {
                broadcast_soil(soilPtr);
                last_sec_viz = sec;
            }

        }

        void update_soil(Soil* soilPtr, float dt) {
            soilPtr->pre_update();

            for(auto & iter : mesh_lookup) {
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

                    std::vector<std::tuple<uint32_t, uint32_t, VertexAttributes *>> footprint;
                    std::vector<std::tuple<uint32_t, uint32_t, VertexAttributes *>> footprint_idx;
                    float total_displaced_volume = 0.0f;

                    #pragma omp parallel num_threads(7) default(none) shared(footprint, total_displaced_volume) firstprivate(footprint_idx, submesh, soilPtr, crot, cpos, pos, rot, indices, dt, link)
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

                            float displaced_volume = 0.0f;
                            footprint_idx = soilPtr->try_deform(meshTri, link, dt, displaced_volume);
                            total_displaced_volume += displaced_volume;

                            /*
                            #pragma omp critical
                            {
                                footprint.insert(footprint.end(), footprint_idx.begin(), footprint_idx.end());
                            }
                             */
                        }
                    }


                    // Footprint level computations
                    /*
                    uint32_t max_x = 0;
                    uint32_t max_y = 0;
                    uint32_t min_x = UINT32_MAX;
                    uint32_t min_y = UINT32_MAX;

                    double w = soilPtr->get_data()->scale;

                    std::unordered_map<uint32_t, std::unordered_map<uint32_t, uint32_t>> footprint_c;

                    std::vector<std::pair<uint32_t,uint32_t>> edge_nodes;

                    for (auto &vtx: footprint) {
                        auto x = std::get<0>(vtx);
                        auto y = std::get<1>(vtx);

                        max_x = std::max(max_x, x);
                        max_y = std::max(max_y, y);
                        min_x = std::min(min_x, x);
                        min_y = std::min(min_y, y);

                        footprint_c[x][y] = 1;
                    }

                    double A = 0.0;
                    double L = 0.0;

                    for (uint32_t x = min_x; x < max_x + 1; x++) {
                        for (uint32_t y = min_y; y < max_y + 1; y++) {
                            int n = 0;
                            if (footprint_c[x][y] == 1) {
                                if (footprint_c[x + 1][y] == 0) {
                                    edge_nodes.emplace_back(x+1,y);
                                    n++;
                                }
                                if (footprint_c[x - 1][y] == 0) {
                                    edge_nodes.emplace_back(x-1,y);
                                    n++;
                                }
                                if (footprint_c[x][y + 1] == 0) {
                                    edge_nodes.emplace_back(x,y+1);
                                    n++;
                                }
                                if (footprint_c[x][y - 1] == 0) {
                                    edge_nodes.emplace_back(x,y-1);
                                    n++;
                                }
                            }
                            if (n == 0) {           // Inner node
                                A += w * w;
                            } else if ( n > 0 ){    // Contour node
                                L += w;
                            }
                        }
                    }


                    float edge_nodes_len = edge_nodes.size();
                    float edge_displacement = 0.5*total_displaced_volume/edge_nodes_len;


                    for(auto& edge : edge_nodes) {

                        auto& x = std::get<0>(edge);
                        auto& y = std::get<1>(edge);
                        auto v3 = soilPtr->get_data()->get_vertex_at_index(x, y)->v3;
                        auto _v3 = Vector3d(v3.X(), v3.Y(), v3.Z() + edge_displacement);

                        soilPtr->get_data()->get_vertex_at_index(x,y)->v3 = _v3;
                        //soilPtr->get_data()->get_vertex_at_index(x,y)->v3_0 = _v3;
                    }
//
//                    auto B = 0.05; //2*A/L;
//                    if(!isnan(B) && !isinf(B)) {
//                        std::cout << B << std::endl;
//                        soilPtr->get_data()->B = B;
//                    } else {
//                        soilPtr->get_data()->B = 0;
//                    }
//                }

                     */

                }
            }
        }


        void broadcast_soil(Soil* soilPtr) {
            hina_ssi_msgs::msgs::Soil soilMsg;
            auto x_w = soilPtr->get_data()->x_width;
            auto y_w = soilPtr->get_data()->y_width;

            Vector3d vert;
            for(int idx = 0; idx < x_w*y_w; idx++) {
                vert = soilPtr->get_data()->vertex_at_flattened_index(idx)->v3;
                soil_v[idx] = msgs::Vector3d();
                soil_v[idx].set_x(vert.X());
                soil_v[idx].set_y(vert.Y());
                soil_v[idx].set_z(vert.Z());
            }

            soilMsg.set_len_col(x_w);
            soilMsg.set_len_row(y_w);

            for(uint32_t i = 0; i < x_w*y_w; i++) {
                auto v = soilMsg.add_flattened_field();
                *v = soil_v[i];
            }
            soilPub->Publish(soilMsg);
        }

    };

    GZ_REGISTER_WORLD_PLUGIN(HinaSSIWorldPlugin)
}

#endif