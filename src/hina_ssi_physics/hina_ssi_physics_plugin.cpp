#ifndef HINA_SSI_PHYSICS_PLUGIN_CPP
#define HINA_SSI_PHYSICS_PLUGIN_CPP

#include <gazebo/common/common.hh>
#include <memory>
#include <gazebo/rendering/rendering.hh>
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

        const siv::PerlinNoise::seed_type seed = 123456u;
        const siv::PerlinNoise perlin{ seed };

        double z;
        Vector3d v3;

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
            init_soil();
            init_transport();
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
            auto _soilData = soilPtr->get_data();
            auto xlen = _soilData.x_width;
            auto ylen = _soilData.y_width;

            for(uint32_t i = 0; i < xlen*ylen; i++) {

                v3 = _soilData.soil_field[i];

                z = 0; //4*perlin.octave2D_01(((v3.X()+sec) * 0.1), (v3.Y() * 0.1), 4);

                _soilData.soil_field[i] = Vector3d(v3.X(), v3.Y(), z);

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