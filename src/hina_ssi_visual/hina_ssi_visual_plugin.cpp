#ifndef HINA_SSI_PLUGIN_CPP
#define HINA_SSI_PLUGIN_CPP

#include <gazebo/common/common.hh>
#include <memory>
#include <gazebo/rendering/rendering.hh>
#include "mesh_generator.cpp"
#include "Soil.pb.h"
#include "../soil.cpp"

namespace gazebo {
    class HinaSSIVisualPlugin : public VisualPlugin {

    private:
        event::ConnectionPtr connectionPtr = nullptr;
        Soil* soil = nullptr;
        MeshGenerator* mesh_gen = nullptr;
        rendering::VisualPtr visual = nullptr;

        transport::NodePtr node = nullptr;
        transport::SubscriberPtr sub = nullptr;
        event::ConnectionPtr updateEventPtr = nullptr;

        bool mesh_initialized = false;

    public:
        HinaSSIVisualPlugin() : VisualPlugin() {
        }

        ~HinaSSIVisualPlugin() override {
            delete soil;
            delete mesh_gen;
        }

        void Load(rendering::VisualPtr _visual, sdf::ElementPtr _sdf) override {
            //connectionPtr = event::Events::ConnectPreRender(boost::bind(&HinaSSIVisualPlugin::update, this));
            init_transport();
            visual = _visual;
        }

        void init_transport() {
            this->node = transport::NodePtr(new transport::Node());
            node->Init();
            sub = node->Subscribe("~/soil", &HinaSSIVisualPlugin::OnSoilUpdate, this);
        }

        void OnSoilUpdate(const boost::shared_ptr<const hina_ssi_msgs::msgs::Soil> &soil_update) {
            if(!mesh_initialized) {
                int x_width = soil_update->len_col();
                int y_width = soil_update->len_row();

                auto* field = new Vector3d[x_width*y_width];
                auto v = soil_update->flattened_field();

                uint32_t i = 0;
                for(const gazebo::msgs::Vector3d& v0 : v) {
                    field[i++] = Vector3d(v0.x(),v0.y(),v0.z());
                }

                soil = new Soil({x_width,y_width,0,0,field});
                init_soil(visual,soil);

                mesh_initialized = true;
            }
        }

        void init_soil(const rendering::VisualPtr& _visual, Soil* soil) {
            mesh_gen = new MeshGenerator();

            auto mesh = mesh_gen->generate_mesh(soil);
            auto scenePtr = _visual->GetScene();
            auto worldViz = scenePtr->WorldVisual();
            auto terrainViz = std::make_shared<rendering::Visual>("terrain_visual", worldViz);

            common::MeshManager::Instance()->AddMesh(mesh);
            scenePtr->AddVisual(terrainViz);
            terrainViz->AttachMesh("terrain_mesh");

        }
    };

    GZ_REGISTER_VISUAL_PLUGIN(HinaSSIVisualPlugin)

}

#endif