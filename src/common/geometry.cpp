#ifndef HINASSI_GEOMETRY_CPP
#define HINASSI_GEOMETRY_CPP

#include <gazebo/common/common.hh>
#include <CGAL/intersections.h>
#include <CGAL/Triangle_2.h>
#include <CGAL/Cartesian.h>
#include <CGAL/Point_2.h>


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
            return Vector3d((v1.X() + v2.X() + v3.X()) / 3,
                               (v1.Y() + v1.Y() + v1.Y()) / 3,
                            (v1.Z() + v1.Z() + v1.Z()) / 3);
        }
    };

    struct AABB {
        Vector3d lu;
        Vector3d ru;
        Vector3d ld;
        Vector3d rd;

        AABB(Vector3d centroid, double dA) {
            lu = Vector3d(centroid.X() - (dA/2), centroid.Y() + (dA/2), centroid.Z());
            ru = Vector3d(centroid.X() + (dA/2), centroid.Y() + (dA/2), centroid.Z());
            ld = Vector3d(centroid.X() - (dA/2), centroid.Y() - (dA/2), centroid.Z());
            rd = Vector3d(centroid.X() + (dA/2), centroid.Y() - (dA/2), centroid.Z());
        }

        auto as_cgal_rect() {
            auto c_lu = Point_2<Kernel>(lu.X(), lu.Y());
            auto c_ru = Point_2<Kernel>(ru.X(), ru.Y());
            auto c_ld = Point_2<Kernel>(ld.X(), ld.Y());
            auto c_rd = Point_2<Kernel>(rd.X(), rd.Y());
            return Iso_rectangle_2<Kernel>(c_lu, c_ru, c_ld, c_rd);
        }
    };

    class CGALGeometry {

    public:
        static bool intersects_projected(Triangle tri, AABB aabb) {
            auto itsx = CGAL::intersection(aabb.as_cgal_rect(), tri.as_cgal_tri_proj());
            if(itsx) {
                return true;
            }
            return false;
        }
    };
}

#endif
