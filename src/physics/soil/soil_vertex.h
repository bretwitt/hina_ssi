#ifndef HINA_SSI_PLUGIN_SOIL_VERTEX_H
#define HINA_SSI_PLUGIN_SOIL_VERTEX_H

#include <gazebo/common/common.hh>
#include <memory>
#include <utility>
#include "../sandbox/sandbox_config.h"
#include "../../common/field/uniform_field.h"

using ignition::math::Vector3d;
using ignition::math::Vector2d;

namespace hina {
struct SoilVertex {

    bool isAir = false;

    /* Frame physics states */
    double plastic_flow{};
    double sigma_yield{};
    double s_p{};
    double s_e{};
    double sigma{};
    Vector3d normal_dA;

    /*
     * 0 = not a node
     * 1 = footprint node
     * 2 = soil deposit node
     * 3 = flow out node
     * */
    int footprint;

    explicit SoilVertex() = default;

    ~SoilVertex() = default;

};
}
#endif //HINA_SSI_PLUGIN_SOIL_VERTEX_H
