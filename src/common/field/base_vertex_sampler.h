#ifndef HINA_SSI_VERTEX_LOADER_H
#define HINA_SSI_VERTEX_LOADER_H

#include <gazebo/common/common.hh>

using ignition::math::Vector2d;

class BaseVertexSampler {
public:
    virtual double get_z_at_index(double x, double y) {
        return 0;
    };
};

#endif //HINA_SSI_VERTEX_LOADER_H
