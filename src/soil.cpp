#ifndef SOIL_CPP
#define SOIL_CPP

#include <gazebo/common/common.hh>
using ignition::math::Vector3d;

namespace gazebo {

    struct SoilData {
        int x_width = 5;
        int y_width = 5;
        double scale = 1.0;
        double x_offset = 0;
        double y_offset = 0;
        Vector3d * soil_field{};

        void init_field() {
            soil_field = new Vector3d[x_width*y_width];
        }

        void setFieldAtIndex(int x, int y, const Vector3d& v) const {
            soil_field[x_width*y + x] = v;
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
            }
        }

        ~Soil() {
            delete[] _data.soil_field;
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

        // bool intersects(ContactMesh* mesh) {}

        bool intersects(Vector3d v1, Vector3d v2, Vector3d v3) {
            // AABB
        }
    };
}

#endif