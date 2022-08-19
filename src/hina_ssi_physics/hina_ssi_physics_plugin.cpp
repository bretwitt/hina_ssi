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
        //std::map<std::string, const common::Mesh*> mesh_lookup {};
        std::map<physics::LinkPtr, const common::Mesh*> mesh_lookup {};
        //std::map<std::string, physics::LinkPtr> link_lookup {};

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
            //load_meshes();
        }

        void OnEntityAdded(const std::string str) {
            std::string links[6] = { "wheel_RR_link", "wheel_FR_link", "wheel_RL_link", "wheel_FL_link", "wheel_R_link", "wheel_L_link" };
            //std::string links[6] = { " ", " ", " ", " ", " ", " " };

            auto model = world->EntityByName(str)->GetParentModel();
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
            soilPtr = new Soil(new SoilData (200,200,0.0025f));
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

            for(std::pair<physics::LinkPtr, const common::Mesh*> pair : mesh_lookup) {
                auto link = pair.first;
                auto mesh = pair.second;

                for(uint32_t i = 0; i < mesh->GetSubMeshCount(); i++) {

                    auto submesh = mesh->GetSubMesh(i);
                    uint32_t indices = submesh->GetIndexCount();

                    auto col_pose = link->GetCollision(0.)->RelativePose();
                    auto cpos = col_pose.Pos();
                    auto crot = col_pose.Rot();

                    auto pose = link->WorldPose();
                    auto rot = pose.Rot();
                    auto pos = pose.Pos();

                    for(uint32_t idx = 0; idx < indices;) {

                        auto v0 = submesh->Vertex(submesh->GetIndex(idx++));
                        auto v1 = submesh->Vertex(submesh->GetIndex(idx++));
                        auto v2 = submesh->Vertex(submesh->GetIndex(idx++));

                        auto cv0 = crot.RotateVector(v0) + cpos;
                        auto cv1 = crot.RotateVector(v1) + cpos;
                        auto cv2 = crot.RotateVector(v2) + cpos;

                        auto c1v0 = rot.RotateVector(cv0) + pos;
                        auto c1v1 = rot.RotateVector(cv1) + pos;
                        auto c1v2 = rot.RotateVector(cv2) + pos;

                        auto meshTri = Triangle(c1v0,c1v1,c1v2);
                        soilPtr->try_deform(meshTri, link, dt);
                    }
                }
                soilPtr->apply_shear_stress(link);
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