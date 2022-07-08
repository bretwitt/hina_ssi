#ifndef HINA_SSI_PLUGIN_CPP
#define HINA_SSI_PLUGIN_CPP

#include <gazebo/common/common.hh>
#include <memory>
#include <gazebo/rendering/rendering.hh>
#include "mesh_generator.cpp"

namespace gazebo {
    class HinaSSIVisualPlugin : public VisualPlugin {

    private:
        event::ConnectionPtr connectionPtr = nullptr;
        Soil* soil = nullptr;
        MeshGenerator* mesh_gen = nullptr;
    public:
        HinaSSIVisualPlugin() : VisualPlugin() {
        }

        ~HinaSSIVisualPlugin() override {
            delete soil;
            delete mesh_gen;
        }

        void Load(rendering::VisualPtr _visual, sdf::ElementPtr _sdf) override {
            //connectionPtr = event::Events::ConnectPreRender(boost::bind(&HinaSSIPlugin::update, this));
            init_soil(_visual);
        }

        void init_soil(const rendering::VisualPtr& _visual) {
            this->mesh_gen = new MeshGenerator();
            this->soil = new Soil();

            auto mesh = mesh_gen->generate_mesh(soil);
            auto scenePtr = _visual->GetScene();
            auto worldViz = scenePtr->WorldVisual();

            auto visual = std::make_shared<rendering::Visual>("terrain_visual", worldViz);
            common::MeshManager::Instance()->AddMesh(mesh);

            scenePtr->AddVisual(visual);
            visual->AttachMesh("terrain_mesh");
        }

        void update_soil(Soil* soil) {
            auto mesh = mesh_gen->generate_mesh(soil);
            auto _mesh = common::MeshManager::Instance()->GetMesh("terrain_mesh");
            if(_mesh == nullptr) {
                return;
            }
            // *_mesh = *mesh
            //*(_mesh->GetSubMesh(0)) = *(mesh->GetSubMesh(0));
            //*(_mesh->GetSubMesh(0)).
        }
    };

    GZ_REGISTER_VISUAL_PLUGIN(HinaSSIVisualPlugin)

}

#endif