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
        Soil* soil = nullptr;
        MeshGenerator* mesh_gen = nullptr;
        common::Mesh* mesh = nullptr;
        Vector3d* field{};

        event::ConnectionPtr connectionPtr = nullptr;
        rendering::VisualPtr visual = nullptr;
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
            delete mesh_gen;
        }

        void Load(rendering::VisualPtr _visual, sdf::ElementPtr _sdf) override {
            connectionPtr = event::Events::ConnectPreRender(boost::bind(&HinaSSIVisualPlugin::update, this));
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

            delete[] field;
            field = new Vector3d[x_width*y_width];

            auto v = soil_update->flattened_field();

            uint32_t i = 0;
            for(const gazebo::msgs::Vector3d& v0 : v) {
                field[i++] = Vector3d(v0.x(),v0.y(),v0.z());
            }
            soil = new Soil({x_width,y_width,0,0,field});

            /*if(soil == nullptr) {*/
//                soil = new Soil({x_width,y_width,0,0,field});
            /*} else {
                for(int i = 0; i < x_width*y_width; i++) {
                    std::cout << field[i] << " " << soil->get_data().soil_field[i] << std::endl;
                }
                std::copy_n(field, x_width*y_width, soil->get_data().soil_field);
            }*/

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
                //std::cout << soil->get_data().soil_field[0].Z() << std::endl;
                update_soil_mesh(soil);
            }
        }

        void init_box() {
            common::MeshManager::Instance()->CreateBox("terrain_mesh", Vector3d(1,1,1), ignition::math::Vector2d(0,0));
            visual->AttachMesh("terrain_mesh");
        }

        void init_soil(Soil* soil) {
            mesh_gen = new MeshGenerator();
            mesh = mesh_gen->generate_mesh(soil);
            common::MeshManager::Instance()->AddMesh(mesh);

            auto scenePtr = visual->GetScene();
            auto worldViz = scenePtr->WorldVisual();
            auto terrainViz = std::make_shared<rendering::Visual>("terrain_visual", worldViz);

            scenePtr->AddVisual(terrainViz);
            terrainViz->AttachMesh("terrain_mesh");
        }

        void update_soil_mesh(Soil* soil) {
            //mesh = mesh_gen->generate_mesh(soil);
            mesh_gen->update_submesh(soil);
        }
    };

    GZ_REGISTER_VISUAL_PLUGIN(HinaSSIVisualPlugin)

}

#endif