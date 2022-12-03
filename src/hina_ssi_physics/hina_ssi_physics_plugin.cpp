#ifndef HINA_SSI_PHYSICS_PLUGIN_CPP
#define HINA_SSI_PHYSICS_PLUGIN_CPP

#include "../common/field/uniform_field.h"
#include <gazebo/common/common.hh>
#include <memory>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/physics/physics.hh>
#include <utility>
#include "soil/soil.h"
#include "dem/dem_loader.h"
#include "Soil.pb.h"

//using namespace hina;

namespace hina {
    class HinaSSIWorldPlugin : public WorldPlugin {

    private:
        std::shared_ptr<Soil> soilPtr = nullptr;
        std::unique_ptr<msgs::Vector3d[]> soil_v = nullptr;

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
        HinaSSIWorldPlugin() : WorldPlugin() {
        }


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
                            }
                        }
                    }
                }
            }
        }

        void init_soil() {
            if (sdf->HasElement("dem")) {
                auto filename = sdf->GetElement("dem")->Get<std::string>();
                init_dem(filename);
            } else {
                init_sandbox();
            }
        }

        void init_sandbox() {
            int x_width = sdf->GetElement("width_x")->Get<int>();
            int y_width = sdf->GetElement("width_y")->Get<int>();
            auto scale = sdf->GetElement("scale")->Get<double>();
            auto angle = sdf->GetElement("angle")->Get<double>();

            soilPtr = std::make_shared<Soil>(SandboxConfig{x_width, y_width, scale, angle});
        }

        void init_dem(const std::string &filename) {
            std::string file_name = gazebo::common::SystemPaths::Instance()->FindFile(filename);
            auto dem = DEMLoader::load_dem(file_name);
            soilPtr = std::make_shared<Soil>(dem);
        }

        void init_transport() {
            auto df = soilPtr->field;
            soil_v = std::make_unique<msgs::Vector3d[]>(df->x_vert_width * df->y_vert_width);
            this->node = transport::NodePtr(new transport::Node());
            node->Init();
            soilPub = node->Advertise<hina_ssi_msgs::msgs::Soil>("~/soil");
        }

        void load_mesh(const physics::LinkPtr &link, const std::string &mesh_uri) {
            auto mesh = common::MeshManager::Instance()->Load(mesh_uri);
            mesh_lookup.insert(std::pair<physics::LinkPtr, const common::Mesh *>(link, mesh));
        }

        void update() {
            time = common::Time::GetWallTime();
            sec = time.Double();

            double dt = sec - last_sec;
            double dt_viz = sec - last_sec_viz;

            update_soil(soilPtr, dt);

            last_sec = sec;

            if (dt_viz > (1. / 5.f)) {
                broadcast_soil(soilPtr);
                last_sec_viz = sec;
            }
        }

        void update_soil(std::shared_ptr<Soil> soil, float dt) {
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

                    std::vector<std::tuple<uint32_t, uint32_t, std::shared_ptr<FieldVertex<SoilAttributes>>>> footprint;
                    std::vector<std::tuple<uint32_t, uint32_t, std::shared_ptr<FieldVertex<SoilAttributes>>>> footprint_idx;
                    float total_displaced_volume = 0.0f;

#pragma omp parallel num_threads(col_threads) default(none) shared(footprint, total_displaced_volume) firstprivate(footprint_idx, submesh, soil, crot, cpos, pos, rot, indices, dt, link)
                    {
#pragma omp for nowait schedule(guided) //reduction(+:total_displaced_volume)
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
                            footprint_idx = soil->try_deform(meshTri, link, dt, displaced_volume);
                            total_displaced_volume += displaced_volume;
                        }
                    }

                }
            }
        }


        void broadcast_soil(std::shared_ptr<Soil> soilPtr) {
            hina_ssi_msgs::msgs::Soil soilMsg;
            auto x_w = soilPtr->field->x_vert_width;
            auto y_w = soilPtr->field->y_vert_width;

            Vector3d vert;
            for (int idx = 0; idx < x_w * y_w; idx++) {
                vert = soilPtr->field->vertex_at_flattened_index(idx)->v3;
                (soil_v)[idx] = msgs::Vector3d();
                (soil_v)[idx].set_x(vert.X());
                (soil_v)[idx].set_y(vert.Y());
                (soil_v)[idx].set_z(vert.Z());
            }

            soilMsg.set_len_col(x_w);
            soilMsg.set_len_row(y_w);

            for (uint32_t i = 0; i < x_w * y_w; i++) {
                auto v = soilMsg.add_flattened_field();
                *v = soil_v[i];
            }
            soilPub->Publish(soilMsg);
        }

    };
    GZ_REGISTER_WORLD_PLUGIN(HinaSSIWorldPlugin)
}
#endif
