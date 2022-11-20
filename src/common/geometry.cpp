#ifndef HINA_SSI_GEOMETRY_H
#define HINA_SSI_GEOMETRY_H

#include "geometry.h"

using namespace gazebo;
using namespace hina;

Geometry* Geometry::INSTANCE = nullptr;

Geometry* Geometry::getInstance() {
    if(!INSTANCE) {
        INSTANCE = new Geometry();
    }
    return INSTANCE;
}

bool Geometry::intersects_box_tri(const Triangle& tri, const AABB& aabb) {
    box_center_bf[0] = static_cast<float>(aabb.center().X());
    box_center_bf[1] = static_cast<float>(aabb.center().Y());

    box_half_size_bf[0] = static_cast<float>(aabb.half_size().X());
    box_half_size_bf[1] = static_cast<float>(aabb.half_size().Y());

    tri_vert_bf[0][0] = static_cast<float>(tri.v1.X());
    tri_vert_bf[0][1] = static_cast<float>(tri.v1.Y());

    tri_vert_bf[1][0] = static_cast<float>(tri.v2.X());
    tri_vert_bf[1][1] = static_cast<float>(tri.v2.Y());
    //tri_vert_bf[1][2] = 0.0;

    tri_vert_bf[2][0] = static_cast<float>(tri.v3.X());
    tri_vert_bf[2][1] = static_cast<float>(tri.v3.Y());

    return triBoxOverlap( box_center_bf, box_half_size_bf, tri_vert_bf);
}

#endif