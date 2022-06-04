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
            //connectionPtr = event::Events::ConnectBeforeCPhysicsUpdate(boost::bind(&HinaSSIPlugin::OnWorldLoad, this));
            init_soil();
        }


        void init_soil() {
            auto mesh_gen = MeshGenerator();
            auto mesh = mesh_gen.generate_mesh();
            auto scenePtr = rendering::get_scene();
            auto visual = std::make_shared<rendering::Visual>("terrain", scenePtr);
            visual->InsertMesh(mesh);
            scenePtr->AddVisual(visual);
        }

    };

    GZ_REGISTER_VISUAL_PLUGIN(HinaSSIPlugin)

}