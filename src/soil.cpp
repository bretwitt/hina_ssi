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
        Vector3d ** soil_field{};

        void init_field() {
            soil_field = new Vector3d*[x_width];
            for(int i = 0; i < x_width; i++) {
                soil_field[i] = new Vector3d[y_width];
            }
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
            for(int i = 0; i < this->_data.x_width; i++) {
                delete this->get_data().soil_field[i];
            }
            delete this->get_data().soil_field;
        }

        SoilData get_data() {
            return _data;
        }

        void reset_soil() {
            _data.x_offset = -(double)_data.x_width / 2;
            _data.y_offset = -(double)_data.y_width / 2;

            for(int i = 0; i < _data.x_width; i++) {
                for(int j = 0; j < _data.y_width; j++) {
                    _data.soil_field[i][j] = Vector3d(i + _data.x_offset,j + _data.y_offset,0.0);
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