#ifndef HINA_SSI_TRIANGLE_CONTEXT_H
#define HINA_SSI_TRIANGLE_CONTEXT_H
#include "../../common/geometry.h"
#include <gazebo/common/common.hh>

namespace hina {
    struct TriangleContext {
        Triangle tri;
        double shear_displacement;
        Vector3d linear_velocity;
        Vector3d slip_velocity;
        double slip = 0;
        Vector3d angular_velocity;
        double B = 0;
    };
}

#endif //HINA_SSI_TRIANGLE_CONTEXT_H
