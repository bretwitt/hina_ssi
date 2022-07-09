#ifndef SOIL_CPP
#define SOIL_CPP

#include <gazebo/common/common.hh>
using ignition::math::Vector3d;

namespace gazebo {

    struct SoilData {
        int x_width = 5;
        int y_width = 5;
        double x_offset = 0;
        double y_offset = 0;
        Vector3d * soil_field{};

        void init_field() {
            soil_field = new Vector3d[x_width*y_width];
        }

        Vector3d getFieldAtIndex(int x, int y) {
            return soil_field[x_width*y + x];
        }

        void setFieldAtIndex(int x, int y, Vector3d v) {
            getFieldAtIndex(x,y) = v;
        }
    };

    class Soil {
    private:
        SoilData _data;

    public:

        Soil(SoilData _data) {
            this->_data = _data;
            this->_data.init_field();
            reset_soil();
        }

        ~Soil() {
            delete _data.soil_field;
        }

        SoilData get_data() {
            return _data;
        }

        void reset_soil() {
            _data.x_offset = -(double)_data.x_width / 2;
            _data.y_offset = -(double)_data.y_width / 2;

            for(int i = 0; i < _data.x_width; i++) {
                for(int j = 0; j < _data.y_width; j++) {
                    _data.setFieldAtIndex(i,j,Vector3d(i + _data.x_offset,j + _data.y_offset,0.0));
                }
            }
        }

        // bool intersects(ContactMesh* mesh) {}

        bool intersects(Vector3d v1, Vector3d v2, Vector3d v3, Vector3d normal) {
            // AABB
        }
    };
}

#endif