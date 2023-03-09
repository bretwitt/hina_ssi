#ifndef HINA_SSI_PLUGIN_SANDBOX_CONFIG_H
#define HINA_SSI_PLUGIN_SANDBOX_CONFIG_H

#include "../soil/soil_physics_params.h"

 namespace hina {
    struct SandboxConfig {

        int x_width;
        int y_width;
        double scale;
        double angle;
        SoilPhysicsParams params;

    };
}


#endif //HINA_SSI_PLUGIN_SANDBOX_CONFIG_H
