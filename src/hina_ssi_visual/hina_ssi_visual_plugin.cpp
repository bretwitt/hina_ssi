#ifndef HINA_SSI_PLUGIN_CPP
#define HINA_SSI_PLUGIN_CPP

#include <gazebo/common/common.hh>
#include <memory>
#include <gazebo/rendering/rendering.hh>
#include "ogre_soil_renderer.cpp"
#include "Soil.pb.h"

namespace gazebo {
    class HinaSSIVisualPlugin : public VisualPlugin {

    private:
        Soil* soil = nullptr;
        OgreSoilRenderer* p_ogre_soil_renderer = nullptr;

        event::ConnectionPtr connectionPtr = nullptr;
        rendering::VisualPtr visual = nullptr;
        rendering::VisualPtr terrainViz = nullptr;
        transport::NodePtr node = nullptr;
        transport::SubscriberPtr sub = nullptr;
        event::ConnectionPtr updateEventPtr = nullptr;


        bool soil_initialized = false;
        bool init_viz = false;

    public:
        HinaSSIVisualPlugin() : VisualPlugin() {
        }

        ~HinaSSIVisualPlugin() override {
            delete soil;
            delete p_ogre_soil_renderer;
        }

        void Load(rendering::VisualPtr _visual, sdf::ElementPtr _sdf) override {
            connectionPtr = event::Events::ConnectRender(boost::bind(&HinaSSIVisualPlugin::update, this));
            init_transport();
            visual = _visual;
        }

        void init_transport() {
            this->node = transport::NodePtr(new transport::Node());
            node->Init();
            sub = node->Subscribe("~/soil", &HinaSSIVisualPlugin::OnSoilUpdate, this);
        }

        void OnSoilUpdate(const boost::shared_ptr<const hina_ssi_msgs::msgs::Soil> &soil_update) {

            int x_width = soil_update->len_col();
            int y_width = soil_update->len_row();

            auto v = soil_update->flattened_field();

            if(soil == nullptr) {
                soil = new Soil(new SoilData({x_width,y_width, 0.005f, 0.0}));
            }

            uint32_t i = 0;
            for(const gazebo::msgs::Vector3d& v0 : v) {
                soil->get_data()->set_vertex_at_flattened_index(i++, VertexAttributes(Vector3d(v0.x(), v0.y(), v0.z())));
            }

            if(!soil_initialized) {
                init_viz = true;
                soil_initialized = true;
            }
        }

        void update() {
            if(init_viz) {
                init_soil(soil);
                init_viz = false;
                return;
            }
            if(soil_initialized) {
                update_soil_mesh(soil);
            }
        }

        void init_soil(Soil* soil) {
            p_ogre_soil_renderer = new OgreSoilRenderer();
            p_ogre_soil_renderer->setScenePtr(visual->GetScene());
            p_ogre_soil_renderer->create_ogre_mesh(soil);
        }

        void update_soil_mesh(Soil* soil) {
            p_ogre_soil_renderer->update_ogre_mesh(soil);
        }
    };

    GZ_REGISTER_VISUAL_PLUGIN(HinaSSIVisualPlugin)

}

#endif