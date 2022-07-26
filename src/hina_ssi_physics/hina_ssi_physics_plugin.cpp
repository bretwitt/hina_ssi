#ifndef HINA_SSI_PHYSICS_PLUGIN_CPP
#define HINA_SSI_PHYSICS_PLUGIN_CPP

#include <gazebo/common/common.hh>
#include <memory>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/physics/physics.hh>
#include <utility>
#include "../common/soil.cpp"
#include "Soil.pb.h"
#include "../../thirdparty/PerlinNoise.h"

namespace gazebo {
    class HinaSSIWorldPlugin : public WorldPlugin {

    private:
        Soil *soilPtr = nullptr;
        msgs::Vector3d *soil_v = nullptr;

        transport::NodePtr node = nullptr;
        transport::PublisherPtr soilPub = nullptr;
        event::ConnectionPtr updateEventPtr = nullptr;
        physics::WorldPtr world = nullptr;
        sdf::ElementPtr sdf = nullptr;

        const siv::PerlinNoise::seed_type seed = 123456u;
        const siv::PerlinNoise perlin{ seed };

        double z;
        Vector3d v3;
        std::map<std::string, const common::Mesh*> mesh_lookup {};
        std::map<std::string, physics::LinkPtr> link_lookup {};

        common::Time time;
        double sec;
        double last_sec;
        double last_sec_viz;

    public:
        HinaSSIWorldPlugin() : WorldPlugin() {
        }

        ~HinaSSIWorldPlugin() override {
            delete soilPtr;
            delete soil_v;
        }

        void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override {
            updateEventPtr = event::Events::ConnectBeforePhysicsUpdate(boost::bind(&HinaSSIWorldPlugin::update, this));
            world = _world;
            sdf = _sdf;
            init_soil();
            init_transport();
            load_meshes();
        }

        void init_soil() {
            soilPtr = new Soil(new SoilData (50,50,0.2f));
        }

        void init_transport() {
            auto df = soilPtr->get_data();
            soil_v = new msgs::Vector3d[df->x_width * df->y_width];
            this->node = transport::NodePtr(new transport::Node());
            node->Init();
            soilPub = node->Advertise<hina_ssi_msgs::msgs::Soil>("~/soil");
        }

        void load_meshes() {
            auto modelElemPtr = sdf->GetParent()->GetElement("model");
            while(modelElemPtr != nullptr) {
                if(!modelElemPtr->HasElement("link")) break;
                auto linkElemPtr = modelElemPtr->GetElement("link");
                while(linkElemPtr != nullptr) {
                    if(!linkElemPtr->HasElement("collision")) break;
                    auto collElemPtr = linkElemPtr->GetElement("collision");
                    if(collElemPtr != nullptr) {
                        if(!collElemPtr->HasElement("geometry")) break;
                        auto geomElemPtr = collElemPtr->GetElement("geometry");
                        if(geomElemPtr != nullptr) {
                            auto meshElemPtr = geomElemPtr->GetElement("mesh");
                            if(!geomElemPtr->HasElement("mesh")) break;
                            if(meshElemPtr != nullptr) {
                                auto uriElemPtr = meshElemPtr->GetElement("uri");
                                if(!meshElemPtr->HasElement("uri")) break;
                                if(uriElemPtr != nullptr) {
                                    auto model = modelElemPtr->GetAttribute("name")->GetAsString();
                                    auto link_name = linkElemPtr->GetAttribute("name")->GetAsString();
                                    auto link = world->ModelByName(model)->GetLink(link_name);
                                    auto uri = uriElemPtr->Get<std::string>();
                                    auto mesh = common::MeshManager::Instance()->Load(uri);

                                    mesh_lookup.insert(std::pair<std::string, const common::Mesh*>(link_name, mesh));
                                    link_lookup.insert(std::pair<std::string, physics::LinkPtr>(link_name, link));
                                }
                            }
                        }
                    }
                    linkElemPtr = linkElemPtr->GetNextElement();
                }
                modelElemPtr = modelElemPtr->GetNextElement();
            }
        }

        void update() {
            time = common::Time::GetWallTime();
            sec = time.Double();
            double dt = sec - last_sec;
            double dt_viz = sec - last_sec_viz;

            if(dt > (1./30)) {
                update_soil(soilPtr);
                last_sec = sec;
            }
            if(dt_viz > (1./5)) {
                broadcast_soil(soilPtr);
                last_sec_viz = sec;
            }
        }

        void update_soil(Soil* soilPtr) {
            for(std::pair<std::string, const common::Mesh*> pair : mesh_lookup) {
                auto linkName = pair.first;
                auto link = link_lookup[linkName];

                auto mesh = pair.second;
                for(uint32_t i = 0; i < mesh->GetSubMeshCount();) {
                    auto submesh = mesh->GetSubMesh(i++);
                    uint32_t indices = submesh->GetIndexCount();
                    for(uint32_t idx = 0; idx < indices;) {
                        auto pose = link->WorldPose();

                        auto v0 = submesh->Vertex(submesh->GetIndex(idx++)) + pose.Pos();
                        auto v1 = submesh->Vertex(submesh->GetIndex(idx++)) + pose.Pos();
                        auto v2 = submesh->Vertex(submesh->GetIndex(idx++)) + pose.Pos();
                        auto meshTri = Triangle(v0, v1, v2);

                        soilPtr->try_deform(meshTri);
                    }
                }
            }
        }


        void broadcast_soil(Soil* soilPtr) {
            hina_ssi_msgs::msgs::Soil soilMsg;
            auto x_w = soilPtr->get_data()->x_width;
            auto y_w = soilPtr->get_data()->y_width;

            Vector3d vert;
            for(int idx = 0; idx < x_w*y_w; idx++) {
                vert = soilPtr->get_data()->vertex_at_flattened_index(idx);
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