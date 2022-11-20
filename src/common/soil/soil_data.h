#ifndef HINA_SSI_PLUGIN_SOIL_DATA_H
#define HINA_SSI_PLUGIN_SOIL_DATA_H

#include <gazebo/common/common.hh>
#include <memory>
#include <utility>
#include "sandbox_config.h"
#include "../field/uniform_field.h"

using ignition::math::Vector3d;
using ignition::math::Vector2d;

namespace gazebo {

    struct SoilAttributes {

        double k_phi = 814000.0f; //814000.0f;
        double k_e = 7.8e7;
        double k_c = 20680.0f;
        double c = 3500;
        double phi = 0.55;

        bool isAir = true;

        /* Frame physics states */
        double plastic_flow{};
        double sigma_yield{};
        double s_p{};
        double s_e{};
        double sigma{};
        Vector3d normal_dA;

        explicit SoilAttributes()  {
        }

        ~SoilAttributes() = default;

    };

}
#endif //HINA_SSI_PLUGIN_SOIL_DATA_H
