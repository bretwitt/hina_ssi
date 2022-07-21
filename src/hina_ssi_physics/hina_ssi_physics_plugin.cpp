#ifndef HINA_SSI_PHYSICS_PLUGIN_CPP
#define HINA_SSI_PHYSICS_PLUGIN_CPP

#include <gazebo/common/common.hh>
#include <memory>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/physics/physics.hh>
#include <utility>
#include "../soil.cpp"
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
        std::map<std::string, const common::Mesh*> mesh_lookup;

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
            soilPtr = new Soil({100,100,0.1f});
        }

        void init_transport() {
            auto df = soilPtr->get_data();
            soil_v = new msgs::Vector3d[df.x_width * df.y_width];
            this->node = transport::NodePtr(new transport::Node());
            node->Init();
            soilPub = node->Advertise<hina_ssi_msgs::msgs::Soil>("~/soil");
        }

        void load_meshes() {
            auto modelElemPtr = sdf->GetParent()->GetElement("model");
            auto link = sdf->GetElement("link");
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
                                    auto uri = uriElemPtr->Get<std::string>();
                                    auto mesh = common::MeshManager::Instance()->Load(uri);
                                    mesh_lookup.insert(std::pair<std::string, const common::Mesh*>(model, mesh));
                                }
                            }
                        }
                    }
                    linkElemPtr = linkElemPtr->GetNextElement();
                }
                modelElemPtr = modelElemPtr->GetNextElement();
            }
        }

//        void init_model_meshes() {
//            auto models = world->Models();
//            for(const auto& model : models) {
//                for(const auto& link : model->GetLinks()) {
//                    for(const auto& collider : link->GetCollisions()) {
//                        auto shape = collider->GetShape();
//                        if(shape->HasType(shape->MESH_SHAPE)) {
//                            init_mesh_shape_uri(shape);
//                        }
//                    }
//                }
//            }
//        }
//
//        void init_mesh_shape_uri(const physics::ShapePtr& shape) {
//            auto meshPtr = boost::dynamic_pointer_cast<physics::MeshShapePtr>(shape);
//            if(meshPtr != nullptr && meshPtr->get() != nullptr) {
//                auto meshURI = meshPtr->get()->GetMeshURI();
//                auto modelName = shape->GetName();
//                model_mesh_uri.insert(std::pair<std::string,std::string>(modelName,meshURI));
//            }
//        }


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
            detect_collisions();
        }

        void detect_collisions() {
            for(std::pair<std::string, const common::Mesh*> pair : mesh_lookup) {
                auto modelName = pair.first;
                auto mesh = pair.second;
                for(uint32_t i = 0; i < mesh->GetSubMeshCount();) {
                    auto submesh = mesh->GetSubMesh(i++);
                    uint32_t indices = submesh->GetIndexCount();
                    for(uint32_t idx = 0; idx < indices;) {
                        auto tri0 = submesh->GetIndex(idx++);
                        auto tri1 = submesh->GetIndex(idx++);
                        auto tri2 = submesh->GetIndex(idx++);
                    }
                }
            }
        }

        void broadcast_soil(Soil* soilPtr) {
            hina_ssi_msgs::msgs::Soil soilMsg;
            auto x_w = soilPtr->get_data().x_width;
            auto y_w = soilPtr->get_data().y_width;

            Vector3d vert;
            for(int idx = 0; idx < x_w*y_w; idx++) {
                vert = soilPtr->get_data().soil_field[idx];
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