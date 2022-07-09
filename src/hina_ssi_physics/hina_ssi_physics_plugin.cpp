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

    public:
        HinaSSIWorldPlugin() : WorldPlugin() {
        }

        ~HinaSSIWorldPlugin() override {
            delete soilPtr;
        }

        void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override {
            updateEventPtr = event::Events::ConnectBeforePhysicsUpdate(boost::bind(&HinaSSIWorldPlugin::update, this));
            init_transport();
            init_soil();
        }

        void init_transport() {
            this->node = transport::NodePtr(new transport::Node());
            node->Init();
            soilPub = node->Advertise<hina_ssi_msgs::msgs::Soil>("~/soil");
        }

        void init_soil() {
            //struct SoilData s{2,2,0,0};
            soilPtr = new Soil({2,2,0,0,{}});
        }

        // TODO: Check timestamp for hard rt freq & dT calculation
        void update() {
            soilPtr = update_soil(soilPtr, 0.0f);
            broadcast_soil(soilPtr);
        }

        Soil* update_soil(Soil* s, float dt) {
            // soil_field = realloc(..);
            return s;
        }

        void broadcast_soil(Soil* soil) {
            hina_ssi_msgs::msgs::Soil soilMsg;
            auto x_w = soil->get_data().x_width;
            auto y_w = soil->get_data().y_width;
            soilMsg.set_len_col(x_w);
            soilMsg.set_len_row(y_w);
            auto *vec_v = new msgs::Vector3d[x_w*y_w];
            for(int i = 0; i < x_w; i++) {
                for(int j = 0; j < y_w; j++) {
                    int idx =  j*x_w + i;
                    auto vert = soil->get_data().soil_field[i][j];

                    vec_v[idx] = msgs::Vector3d();
                    vec_v[idx].set_x(vert.X());
                    vec_v[idx].set_y(vert.Y());
                    vec_v[idx].set_z(vert.Z());
                }
            }
            auto v = soilMsg.add_flattened_field();
            *v = *vec_v;
            soilPub->Publish(soilMsg);
        }

    };

    GZ_REGISTER_WORLD_PLUGIN(HinaSSIWorldPlugin)
}

#endif