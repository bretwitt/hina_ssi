#ifndef HINASSI_GEOMETRY_CPP
#define HINASSI_GEOMETRY_CPP

#include <gazebo/common/common.hh>
#include <CGAL/intersections.h>
#include <CGAL/Triangle_2.h>
#include <CGAL/Cartesian.h>
#include <CGAL/Point_2.h>
#include "../../thirdparty/MollerTest.h"

using ignition::math::Vector3d;
using ignition::math::Vector2d;
using namespace CGAL;
typedef CGAL::Cartesian<double> Kernel;

namespace gazebo {

    struct Triangle {
        Vector3d v1;
        Vector3d v2;
        Vector3d v3;

        Triangle(Vector3d v1, Vector3d v2, Vector3d v3) {
            this->v1 = v1;
            this->v2 = v2;
            this->v3 = v3;
        }

        auto as_cgal_p3(Vector3d v) {
            return Point_3<Kernel>(v.X(), v.Y(), v.Z());
        }

        auto as_cgal_p2(Vector3d v) {
            return Point_2<Kernel>(v.X(), v.Y());
        }

        auto as_cgal_tri() {
            auto t3 = Triangle_3<Kernel>(as_cgal_p3(v1), as_cgal_p3(v2), as_cgal_p3(v3));
            return t3;
        }

        auto as_cgal_tri_proj() {
            auto t2 = Triangle_2<Kernel>(as_cgal_p2(v1), as_cgal_p2(v2), as_cgal_p2(v3));
            return t2;
        }

        Vector3d centroid() {
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
