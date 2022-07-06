#include <gazebo/common/common.hh>
#include <memory>
#include <gazebo/rendering/rendering.hh>
#include "MeshGenerator.cpp"

namespace gazebo {
    class HinaSSIPlugin : public VisualPlugin {

    private:
        event::ConnectionPtr connectionPtr = nullptr;
    public:
        HinaSSIPlugin() : VisualPlugin() {
        }

        void Load(rendering::VisualPtr _visual, sdf::ElementPtr _sdf) {
            //connectionPtr = event::Events::ConnectPreRender(boost::bind(&HinaSSIPlugin::update, this));
            init_soil(_visual);
        }


        void init_soil(const rendering::VisualPtr& _visual) {
            auto mesh_gen = MeshGenerator();
            auto mesh = mesh_gen.generate_mesh();
            auto scenePtr = _visual->GetScene();
            auto worldViz = scenePtr->WorldVisual();
            auto visual = std::make_shared<rendering::Visual>("terrain_visual", worldViz);
            common::MeshManager::Instance()->AddMesh(mesh);

            scenePtr->AddVisual(visual);
            visual->AttachMesh("terrain_mesh");
        }

    };

    GZ_REGISTER_VISUAL_PLUGIN(HinaSSIPlugin)

}