#ifndef HINA_SSI_PLUGIN_SOIL_VERTEX_H
#define HINA_SSI_PLUGIN_SOIL_VERTEX_H

#include <gazebo/common/common.hh>
#include <memory>
#include <utility>

#include "../../common/field/field.h"

using ignition::math::Vector3d;
using ignition::math::Vector2d;

namespace hina {
struct SoilVertex {

    bool isAir = false;

    /* Frame physics states */
    double sinkage{};
    double sigma{};
    Vector3d normal_dA{};
    Vector3d normal = Vector3d(0,0,1);

    /*
     * 0 = not a node
     * 1 = footprint node
     * 2 = soil deposit node
     * 3 = flow out node
     * */
    int footprint = 0;

    explicit SoilVertex() = default;

    ~SoilVertex() = default;

};
}
#endif //HINA_SSI_PLUGIN_SOIL_VERTEX_H
