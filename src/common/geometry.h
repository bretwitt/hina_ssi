#ifndef HINA_SSI_PLUGIN_GEOMETRY_H
#define HINA_SSI_PLUGIN_GEOMETRY_H

#include <gazebo/common/common.hh>
#include "../../thirdparty/MollerTest.h"

using ignition::math::Vector3d;
using ignition::math::Vector2d;

struct Triangle {
    Vector3d v1;
    Vector3d v2;
    Vector3d v3;

    Triangle(const Vector3d& v1, const Vector3d& v2, const Vector3d& v3) {
        this->v1 = v1;
        this->v2 = v2;
        this->v3 = v3;
    }

    Vector3d centroid() const {
        return {(v1.X() + v2.X() + v3.X()) / 3,
                (v1.Y() + v1.Y() + v1.Y()) / 3,
                (v1.Z() + v1.Z() + v1.Z()) / 3 };
    }
};

struct AABB {
    Vector3d lu;
    Vector3d ru;
    Vector3d ld;
    Vector3d rd;
    double center_x;
    double center_y;
    double dA_half;

    AABB(Vector3d centroid, double dA) {
        lu = Vector3d(centroid.X() - (dA / 2), centroid.Y() + (dA / 2), 0);
        ru = Vector3d(centroid.X() + (dA / 2), centroid.Y() + (dA / 2), 0);
        ld = Vector3d(centroid.X() - (dA / 2), centroid.Y() - (dA / 2), 0);
        rd = Vector3d(centroid.X() + (dA / 2), centroid.Y() - (dA / 2), 0);

        dA_half = dA / 2;
        center_x = (lu.X() + ru.X()) * 0.5;
        center_y = (lu.Y() + ru.Y()) * 0.5;
    }

    Vector2d center() const {
        return {center_x, center_y};
    }

    Vector2d half_size() const {
        return {dA_half, dA_half};
    }
};

class Geometry {
private:
    static Geometry* INSTANCE;

public:
    float box_center_bf[3] = { 0, 0, 0 };
    float box_half_size_bf[3] = { 0, 0, 0 };
    float tri_vert_bf[3][3] = { { 0, 0, 0 }, { 0,0,0 }, { 0, 0, 0}};


    static Geometry* getInstance();

    bool intersects_box_tri(const Triangle& tri, const AABB& aabb);
};

#endif //HINA_SSI_PLUGIN_GEOMETRY_H