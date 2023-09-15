#ifndef HINA_SSI_BODYPHYSICS_H
#define HINA_SSI_BODYPHYSICS_H

#include <gazebo/common/common.hh>

struct VisualBodyPhysics {
    std::vector<gazebo::msgs::Vector3d> traction;
    std::vector<gazebo::msgs::Vector3d> normal;
    std::vector<gazebo::msgs::Vector3d> origin;
};

#endif //HINA_SSI_BODYPHYSICS_H
