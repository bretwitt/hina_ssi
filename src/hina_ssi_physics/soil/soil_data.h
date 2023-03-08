#ifndef HINA_SSI_PLUGIN_SOIL_DATA_H
#define HINA_SSI_PLUGIN_SOIL_DATA_H

#include <gazebo/common/common.hh>
#include <memory>
#include <utility>
#include "../sandbox/sandbox_config.h"
#include "../../common/field/uniform_field.h"

using ignition::math::Vector3d;
using ignition::math::Vector2d;

namespace hina {
struct SoilAttributes {

    bool isAir = false;

    /* Frame physics states */
    double plastic_flow{};
    double sigma_yield{};
    double s_p{};
    double s_e{};
    double sigma{};
    Vector3d normal_dA;

    explicit SoilAttributes() = default;

    ~SoilAttributes() = default;

};
}
#endif //HINA_SSI_PLUGIN_SOIL_DATA_H
