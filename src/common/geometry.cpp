#ifndef HINASSI_GEOMETRY_CPP
#define HINASSI_GEOMETRY_CPP

#include <gazebo/common/common.hh>
#include "../../thirdparty/MollerTest.h"

using ignition::math::Vector3d;
using ignition::math::Vector2d;

namespace gazebo {

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
        double dA;

        AABB(Vector3d centroid, double dA) {
            this->dA = dA;
            lu = Vector3d(centroid.X() - (dA/2), centroid.Y() + (dA/2), 0);
            ru = Vector3d(centroid.X() + (dA/2), centroid.Y() + (dA/2), 0);
            ld = Vector3d(centroid.X() - (dA/2), centroid.Y() - (dA/2), 0);
            rd = Vector3d(centroid.X() + (dA/2), centroid.Y() - (dA/2), 0);
        }

        Vector2d center() {
            return {(lu.X() + ru.X()) * 0.5, (lu.Y() + ld.Y()) * 0.5 };
        }

        Vector2d half_size() const {
            return { dA/2, dA/2 };
        }
    };

    class Geometry {
    public:
        static bool intersects_box_tri(Triangle tri, AABB aabb) {
            float boxCenter[3] = { static_cast<float>(aabb.center().X()), static_cast<float>(aabb.center().Y()), 0.0 };
            float boxHalfSize[3] = { static_cast<float>(aabb.half_size().X()), static_cast<float>(aabb.half_size().Y()), 0.0 };
            float triVerts[3][3] =
                    {
                        { static_cast<float>(tri.v1.X()), static_cast<float>(tri.v1.Y()), 0 },
                        { static_cast<float>(tri.v2.X()), static_cast<float>(tri.v2.Y()), 0 },
                        { static_cast<float>(tri.v3.X()), static_cast<float>(tri.v3.Y()), 0 }
                    };

            return triBoxOverlap( boxCenter, boxHalfSize, triVerts);
        }
    };
}

#endif
