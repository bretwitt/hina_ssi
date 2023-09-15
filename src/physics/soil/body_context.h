#ifndef HINA_SSI_BODY_CONTEXT_H
#define HINA_SSI_BODY_CONTEXT_H
#include "../../common/geometry.h"
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>

struct BodyContext {
    Vector3d traction;
    Vector3d normal;
    Vector3d origin;
};

#endif //HINA_SSI_BODY_CONTEXT_H
