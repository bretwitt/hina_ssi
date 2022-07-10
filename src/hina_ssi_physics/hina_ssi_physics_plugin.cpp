#ifndef HINA_SSI_PHYSICS_PLUGIN_CPP
#define HINA_SSI_PHYSICS_PLUGIN_CPP

#include <gazebo/common/common.hh>
#include <memory>
#include <gazebo/rendering/rendering.hh>
#include "../soil.cpp"
#include "Soil.pb.h"

namespace gazebo {
    class HinaSSIWorldPlugin : public WorldPlugin {

    private:
        Soil *soilPtr = nullptr;

        transport::NodePtr node = nullptr;
        transport::PublisherPtr soilPub = nullptr;
        event::ConnectionPtr updateEventPtr = nullptr;

        msgs::Vector3d *soil_v = nullptr;

    public:
        HinaSSIWorldPlugin() : WorldPlugin() {
        }

        ~HinaSSIWorldPlugin() override {
            delete soilPtr;
            delete soil_v;
        }

        void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override {
            updateEventPtr = event::Events::ConnectBeforePhysicsUpdate(boost::bind(&HinaSSIWorldPlugin::update, this));
            init_soil();
            init_transport();
        }

        void init_soil() {
            soilPtr = new Soil({50,50,0,0});
        }

        void init_transport() {
            auto df = soilPtr->get_data();
            soil_v = new msgs::Vector3d[df.x_width * df.y_width];
            this->node = transport::NodePtr(new transport::Node());
            node->Init();
            soilPub = node->Advertise<hina_ssi_msgs::msgs::Soil>("~/soil");
        }


        // TODO: Check timestamp for hard rt freq & dT calculation
        void update() {
            //soilPtr = update_soil(soilPtr, 0.0f);
            broadcast_soil(soilPtr);
        }

        void broadcast_soil(Soil* soil) {
            hina_ssi_msgs::msgs::Soil soilMsg;
            auto x_w = soil->get_data().x_width;
            auto y_w = soil->get_data().y_width;
            soilMsg.set_len_col(x_w);
            soilMsg.set_len_row(y_w);
            for(int i = 0; i < x_w; i++) {
                for(int j = 0; j < y_w; j++) {
                    int idx =  j*x_w + i;
                    auto vert = soil->get_data().getFieldAtIndex(i,j);
                    soil_v[idx] = msgs::Vector3d();
                    soil_v[idx].set_x(vert.X());
                    soil_v[idx].set_y(vert.Y());
                    soil_v[idx].set_z(vert.Z());
                }
            }
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