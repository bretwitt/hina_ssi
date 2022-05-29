#include <gazebo/gazebo.hh>

namespace gazebo {
    class HinaSSIPlugin : public WorldPlugin {

    public:
        HinaSSIPlugin() : WorldPlugin() {

        }

        void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) {
        }

    };
    GZ_REGISTER_WORLD_PLUGIN(HinaSSIPlugin)
}