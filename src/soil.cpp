#ifndef SOIL_CPP
#define SOIL_CPP

#include <gazebo/common/common.hh>
#include <CGAL/intersections.h>
#include <CGAL/Triangle_2.h>
#include <CGAL/Cartesian.h>

using ignition::math::Vector3d;
using namespace CGAL;
typedef CGAL::Cartesian<double> Kernel;

namespace gazebo {

    struct SoilData {
        int x_width = 5;
        int y_width = 5;
        double scale = 1.0;
        double x_offset = 0;
        double y_offset = 0;
        Vector3d * soil_field{};
        uint32_t * indices{};

        void init_field() {
            soil_field = new Vector3d[x_width*y_width];
            indices = new uint32_t[(x_width - 1)*(y_width - 1)*3*2];
        }

        void setFieldAtIndex(int x, int y, const Vector3d& v) const {
            soil_field[x_width*y + x] = v;
        }
    };

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
            return Point_3<Kernel>(v1.X(), v1.Y(), v1.Z());
        }

        auto as_cgal_tri() {
            return Triangle_3<Kernel>(as_cgal_p3(v1),as_cgal_p3(v2),as_cgal_p3(v3));
        }
    };

    class Soil {
    private:
        SoilData _data;

    public:

        explicit Soil(SoilData soil_data) {
            this->_data = soil_data;
            if(_data.soil_field == nullptr) {
                _data.init_field();
                generate_soil_vertices();
                generate_indices();
            }
        }

        ~Soil() {
            delete[] _data.soil_field;
            delete[] _data.indices;
        }

        SoilData get_data() {
            return _data;
        }

        void generate_soil_vertices() {
            _data.x_offset = -(double)_data.x_width / 2;
            _data.y_offset = -(double)_data.y_width / 2;

            for(int i = 0; i < _data.x_width; i++) {
                for(int j = 0; j < _data.y_width; j++) {
                    auto i_f = (float)i;
                    auto j_f = (float)j;
                    auto v3 = Vector3d(_data.scale*(i_f + _data.x_offset),_data.scale*(j_f + _data.y_offset),0.0);
                    _data.setFieldAtIndex(i,j,v3);
                }
            }
        }

        void generate_indices() {
            uint32_t x_size = _data.x_width;
            uint32_t y_size = _data.y_width;
            auto indices = _data.indices;

            uint32_t idx = 0;
            for(uint32_t y = 0; y < y_size - 1; y++) {
                for(uint32_t x = 0; x < x_size - 1; x++) {
                    uint32_t a = (x_size * x) + y;
                    uint32_t b = (x_size * (x + 1)) + y;
                    uint32_t c = (x_size * (x + 1)) + (y + 1);
                    uint32_t d = (x_size * x) + (y + 1);

                    indices[idx++] = a;
                    indices[idx++] = d;
                    indices[idx++] = c;
                    indices[idx++] = c;
                    indices[idx++] = b;
                    indices[idx++] = a;
                }
            }
        }

        //bool intersects(ContactMesh* mesh) {}

        bool intersects(Triangle meshTri) {
            uint32_t idx = 0;
            auto x_size = _data.x_width;
            auto y_size = _data.y_width;
            auto indices = _data.indices;
            auto verts = _data.soil_field;

            for(uint32_t n = 0; n < (x_size - 1)*(y_size - 1)*2; n++) {
                auto p0 = verts[indices[idx++]];
                auto p1 = verts[indices[idx++]];
                auto p2 = verts[indices[idx++]];
                Triangle soilTri(p0,p1,p2);
                if(intersects(meshTri,soilTri)) {
                    return true;
                }
            }
            return false;
        }

        bool intersects(Triangle meshTri, Triangle soilTri) {
            auto intersection = CGAL::intersection(meshTri.as_cgal_tri(),soilTri.as_cgal_tri());
            return !intersection->empty();
        }
    };
}

#endif