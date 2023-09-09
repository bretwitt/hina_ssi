#ifndef HINA_SSI_VISUAL_TRIANGLES_H
#define HINA_SSI_VISUAL_TRIANGLES_H
#include <gazebo/common/common.hh>
#include <vector>

namespace hina {
    struct VisualTriangles {
        std::vector<gazebo::msgs::Vector3d> centroid;
        std::vector<gazebo::msgs::Vector3d> forces;
        std::vector<gazebo::msgs::Vector3d> slip_velocity;
        std::vector<gazebo::msgs::Vector3d> normal;
        std::vector<bool> contact;
        std::vector<double> shear_displacement;

    };
}

#endif //HINA_SSI_VISUAL_TRIANGLES_H
