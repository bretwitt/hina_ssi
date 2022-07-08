#ifndef HINA_SSI_PHYSICS_PLUGIN_CPP
#define HINA_SSI_PHYSICS_PLUGIN_CPP

#include <gazebo/common/common.hh>
#include <memory>
#include <gazebo/rendering/rendering.hh>
#include "../soil.cpp"
#include "Soil.pb.h"

namespace gazebo {
    class HinaSSIWorldPlugin : public WorldPlugin {

    public:
        transport::NodePtr node = nullptr;
        transport::SubscriberPtr sub = nullptr;
        transport::PublisherPtr soilPub = nullptr;
        event::ConnectionPtr updateEventPtr = nullptr;
        Soil* soil_field = nullptr;

        HinaSSIWorldPlugin() : WorldPlugin() {
        }

        void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override {
            std::cout << "Loaded World Plugin" << std::endl;
            init_node();
        }

        void init_node() {
            this->node = transport::NodePtr(new transport::Node());
            node->Init();
            updateEventPtr = event::Events::ConnectBeforePhysicsUpdate(boost::bind(&HinaSSIWorldPlugin::update, this));
            soilPub = node->Advertise<hina_ssi_msgs::msgs::Soil>("~/soil");
        }

        void update() {
            //soil_field = update_soil();
            //broadcast_soil(soil_field);
        }

        void update_soil() {

        }

        void broadcast_soil(Soil* soil) {
            msgs::GzString str;
            str.set_data("str");
            soilPub->Publish(str);
        }

    };

    GZ_REGISTER_WORLD_PLUGIN(HinaSSIWorldPlugin)
}

#endif