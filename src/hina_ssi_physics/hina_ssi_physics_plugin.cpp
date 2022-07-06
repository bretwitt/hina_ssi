#ifndef HINA_SSI_PHYSICS_PLUGIN_CPP
#define HINA_SSI_PHYSICS_PLUGIN_CPP

#include <gazebo/common/common.hh>
#include <memory>
#include <gazebo/rendering/rendering.hh>

namespace gazebo {
    class HinaSSIWorldPlugin : public WorldPlugin {

    public:
        HinaSSIWorldPlugin() : WorldPlugin() {
        }
        void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) {
        }
    };

    GZ_REGISTER_WORLD_PLUGIN(HinaSSIWorldPlugin)
}

#endif